/*  proxsim
 *  Simulator.cpp
 *
 *  Copyright (c) 2009, Ewen Cheslack-Postava
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of libprox nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Simulator.hpp"
#include <stdint.h>
#include <algorithm>
#include "CSVLoader.hpp"

#define RATE_APPROX_ITERATIONS 100

namespace Prox {
namespace Simulation {

static float randFloat() {
    return float(rand()) / RAND_MAX;
}

static uint32 randUInt32(uint32 minval, uint32 maxval) {
    uint32 r = (uint32)( randFloat() * (maxval-minval+1) + minval);
    if (r > maxval) r = maxval;
    return r;
}

Simulator::Simulator(QueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime)
 : mFinished(false),
   mDuration(duration),
   mTimestep(timestep),
   mIterations(0),
   mTerminateIterations(iterations),
   mRealtime(realtime),
   mTime(Time::null()),
   mObjectIDSource(0),
   mHandler(handler),
   mLocCache(NULL),
   mHaveMovingObjects(false),
   mReportRate(false),
   mItsSinceRateApprox(0),
   mRateApproxStart(0)
{
}

Simulator::~Simulator() {
}

static Vector3 generatePosition(const BoundingBox3& region) {
    Vector3 region_min = region.min();
    Vector3 region_extents = region.extents();

    return region_min + Vector3(region_extents.x * randFloat(), region_extents.y * randFloat(), 0.f/*region_extents.z * randFloat()*/);
}

static Vector3 generateDirection(bool moving) {
    return
        (moving ?
            Vector3(randFloat() * 20.f - 10.f, randFloat() * 20.f - 10.f, 0.f/*randFloat() * 20.f - 10.f*/) :
            Vector3(0, 0, 0)
        );
};

static MotionPath generateMotionPath(const BoundingBox3& region, bool moving) {
    // Our simple model is ballistic, single update

    Vector3 offset = generatePosition(region);
    MotionPath::MotionVectorListPtr updates(
        new MotionPath::MotionVectorList()
    );
    Vector3 dir = generateDirection(moving);
    updates->push_back(
        MotionVector3(
            Time::null(),
            Vector3(0,0,0),
            dir
        )
    );

    return MotionPath(offset, updates);
}

static BoundingBox3 generateObjectBounds() {
    return BoundingBox3( Vector3(-1, -1, -1), Vector3(1, 1, 1));
};

static BoundingBox3 generateQueryBounds() {
    return BoundingBox3( Vector3(0, 0, 0), Vector3(0, 0, 0) );
}

static float generateQueryRadius() {
    static float val = sqrtf(6.f) / 2.f; // Diagonal of bounding box
    return val;
}

static SolidAngle generateQueryAngle() {
    return SolidAngle( SolidAngle::Max / 1000 );
}

static Querier* generateQuery(QueryHandler* handler, const BoundingBox3& region, bool static_queries) {
    Vector3 qpos = generatePosition(region);
    Vector3 qvel = generateDirection(!static_queries);

    MotionPath::MotionVectorListPtr mpl(new MotionPath::MotionVectorList());
    mpl->push_back(MotionVector3(Time::null(), qpos, qvel));

    Querier* querier = new Querier(handler,
        MotionPath(Vector3(0,0,0), mpl),
        generateQueryBounds(),
        generateQueryRadius(),
        generateQueryAngle()
    );
    return querier;
}

void Simulator::initialize(int churnrate) {
    mChurn = churnrate;

    ObjectLocationServiceCache* loc_cache = new ObjectLocationServiceCache();
    addListener(loc_cache);
    mLocCache = loc_cache;

    mHandler->initialize(mLocCache, mHaveMovingObjects);
}

void Simulator::createRandomObjects(const BoundingBox3& region, int nobjects, float moving_frac) {
    // Generate objects
    mHaveMovingObjects = mHaveMovingObjects || (moving_frac > 0.0);
    for(int i = 0; i < nobjects; i++) {
        mObjectIDSource++;
        unsigned char oid_data[ObjectID::static_size]={0};
        memcpy(oid_data,&mObjectIDSource,ObjectID::static_size<sizeof(mObjectIDSource)?ObjectID::static_size:sizeof(mObjectIDSource));
        ObjectID oid(oid_data,ObjectID::static_size);

        bool moving = (randFloat() < moving_frac);

        Object* obj = new Object(
            ObjectID(oid_data,ObjectID::static_size),
            generateMotionPath(region, moving),
            generateObjectBounds()
        );

        mRegion.mergeIn( BoundingBox3(obj->position(Time::null()), obj->position(Time::null())) );

        mAllObjects[obj->id()] = obj;
        mRemovedObjects[obj->id()] = obj;
    }
}

void Simulator::createStaticCSVObjects(const std::string csvfile, int nobjects) {
    std::vector<Object*> objects = loadCSVObjects(csvfile);
    createCSVObjects(objects, nobjects);
}

void Simulator::createMotionCSVObjects(const std::string csvfile, int nobjects) {
    mHaveMovingObjects = true;
    std::vector<Object*> objects =
        loadCSVMotionObjects(
            csvfile,
            std::tr1::bind(generatePosition, mRegion)
        );
    createCSVObjects(objects, nobjects);
}

void Simulator::createCSVObjects(std::vector<Object*>& objects, int nobjects) {
    nobjects = std::min(nobjects, (int)objects.size()); // just in case we don't have enough
    // Sample a subset of the objects
    for (int i = 0; i < nobjects; i++) {
        int x = rand() % objects.size();
        Object* obj = objects[x];
        objects.erase( objects.begin() + x );
        mRegion.mergeIn( BoundingBox3(obj->position(Time::null()), obj->position(Time::null())) );
        mAllObjects[obj->id()] = obj;
        mRemovedObjects[obj->id()] = obj;
    }
    // Get rid of the leftovers
    for(int i = 0; i < objects.size(); i++)
        delete objects[i];
}

void Simulator::createRandomQueries(int nqueries, bool static_queries) {
    for(int i = 0; i < nqueries; i++)
        addQuery( generateQuery(mHandler, mRegion, static_queries) );
}

void Simulator::createCSVQueries(int nqueries, const std::string& csvmotionfile) {
    float qradius = generateQueryRadius();
    SolidAngle qangle = generateQueryAngle();

    std::vector<Querier*> qs =
        loadCSVMotionQueriers(
            csvmotionfile, nqueries,
            mHandler,
            std::tr1::bind(generatePosition, mRegion),
            qradius, qangle
        );
    for(int i = 0; i < qs.size(); i++)
        addQuery(qs[i]);
}

void Simulator::addObjects() {
    // Add objects
    int32 count = 0;
    for(ObjectList::iterator it = mAllObjects.begin(); it != mAllObjects.end(); it++) {
        count++;
        if (count > mChurn)
            addObject(it->second);
    }
}

void Simulator::run() {
    addObjects();

    mTimer.start();
}

void Simulator::shutdown() {
    // Remove all objects
    while(!mObjects.empty()) {
        Object* obj = mObjects.begin()->second;
        removeObject(obj);
    }
    // And delete them all
    while(!mRemovedObjects.empty()) {
        Object* obj = mRemovedObjects.begin()->second;
        delete obj;
        mRemovedObjects.erase(mRemovedObjects.begin());
    }

    while(!mQueries.empty()) {
        Querier* query = mQueries.front();
        removeQuery(query);
        delete query;
    }

    delete mLocCache;
}

const BoundingBox3& Simulator::region() const {
    return mRegion;
}

void Simulator::addListener(SimulatorListener* listener) {
    assert( std::find(mListeners.begin(), mListeners.end(), listener) == mListeners.end() );
    mListeners.push_back(listener);
}

void Simulator::removeListener(SimulatorListener* listener) {
    ListenerList::iterator it = std::find(mListeners.begin(), mListeners.end(), listener);
    assert( it != mListeners.end() );
    mListeners.erase(it);
}

void Simulator::tick() {
    Duration elapsed = mTimer.elapsed();
    if (mRealtime) {
        if (mDuration > 0 && elapsed.seconds() > mDuration) {
            mFinished = true;
            return;
        }
        mTime = Time::null() + elapsed;
    }
    else {
        mTime += mTimestep;
        if (mDuration > 0 && ((mTime - Time::null()).seconds() > mDuration)) {
            mFinished = true;
            return;
        }
    }

    //fprintf(stderr, "Tick: %f\n", (mTime - Time::null()).seconds());

    // Give all objects a chance to update their positions
    for(ObjectList::iterator it = mAllObjects.begin(); it != mObjects.end(); it++)
        it->second->tick(mTime);

    // Object Churn...
    for(int i = 0; !mObjects.empty() && i < mChurn; i++) {
        Object* obj = mObjects.begin()->second;
        removeObject(obj);
    }
    for(int i = 0; !mRemovedObjects.empty() && i < mChurn; i++) {
        Object* obj = mRemovedObjects.begin()->second;
        addObject(obj);
    }

    // Give all queries a chance to update
    for(QueryList::iterator it = mQueries.begin(); it != mQueries.end(); it++)
        (*it)->tick(mTime);

    // Tick the handler
    mHandler->tick(mTime);

    mItsSinceRateApprox++;
    if (mItsSinceRateApprox >= RATE_APPROX_ITERATIONS) {
        float its_per_sec = float(RATE_APPROX_ITERATIONS) * queriesSize() / (elapsed - mRateApproxStart).seconds();
        if (mReportRate)
            printf("{ \"rate\" : %f }\n", its_per_sec);
        // Reset for next round
        mItsSinceRateApprox = 0;
        mRateApproxStart = elapsed;
    }

    mIterations++;
    if (mTerminateIterations > 0 && mIterations >= mTerminateIterations)
        mFinished = true;
}

void Simulator::addObject(Object* obj) {
    mRemovedObjects.erase( mRemovedObjects.find(obj->id()) );
    mObjects[obj->id()] = obj;
    mLocCache->addObject(obj);
    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorAddedObject(obj, obj->position(), obj->bounds());
}

void Simulator::removeObject(Object* obj) {
    ObjectList::iterator it = mObjects.find(obj->id());
    mObjects.erase(it);
    mRemovedObjects[obj->id()] = obj;
    mLocCache->removeObject(obj);
    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorRemovedObject(obj);
}

void Simulator::addQuery(Querier* query) {
    mQueries.push_back(query);
    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorAddedQuery(query);
}

void Simulator::removeQuery(Querier* query) {
    QueryList::iterator it = std::find(mQueries.begin(), mQueries.end(), query);
    mQueries.erase(it);

    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorRemovedQuery(query);
}

Simulator::ObjectIterator Simulator::objectsBegin() {
    return mObjects.begin();
}

Simulator::ObjectIterator Simulator::objectsEnd() {
    return mObjects.end();
}

Simulator::ObjectIterator Simulator::objectsFind(const ObjectID& objid) {
    return mObjects.find(objid);
}

int Simulator::objectsSize() const {
    return mObjects.size();
}

int Simulator::allObjectsSize() const {
    return mAllObjects.size();
}

Simulator::QueryIterator Simulator::queriesBegin() {
    return mQueries.begin();
}

Simulator::QueryIterator Simulator::queriesEnd() {
    return mQueries.end();
}

uint32 Simulator::queriesSize() const {
    return mQueries.size();
}

} // namespace Simulation
} // namespace Prox
