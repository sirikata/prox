// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "SimulatorBase.hpp"
#include <stdint.h>
#include <algorithm>
#include "CSVLoader.hpp"

#define RATE_APPROX_ITERATIONS 25

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

SimulatorBase::SimulatorBase(int duration, const Duration& timestep, int iterations, bool realtime)
 : mFinished(false),
   mDuration(duration),
   mTimestep(timestep),
   mIterations(0),
   mTerminateIterations(iterations),
   mRealtime(realtime),
   mTime(Time::null()),
   mObjectIDSource(0),
   mLocCache(NULL),
   mHaveMovingObjects(false)
{
}

SimulatorBase::~SimulatorBase() {
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

static SolidAngle generateQueryAngle(const SolidAngle& qmin, const SolidAngle& qmax) {
    assert(qmax >= qmin);
    if (qmax == qmin) return qmin;

    return qmin + ((qmax-qmin) * (((float)(rand()))/RAND_MAX));
}

void SimulatorBase::initialize(int churnrate) {
    mChurn = churnrate;
}

void SimulatorBase::createRandomObjects(const BoundingBox3& region, int nobjects, float moving_frac) {
    // Generate objects
    mHaveMovingObjects = mHaveMovingObjects || (moving_frac > 0.0 && nobjects > 0);
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
        if (obj->dynamic())
            mRemovedDynamicObjects[obj->id()] = obj;
        else
            mRemovedStaticObjects[obj->id()] = obj;
    }
}

void SimulatorBase::createStaticCSVObjects(const std::string csvfile, int nobjects) {
    std::vector<Object*> objects = loadCSVObjects(csvfile);
    createCSVObjects(objects, nobjects);
}

void SimulatorBase::createMotionCSVObjects(const std::string csvfile, int nobjects) {
    mHaveMovingObjects = mHaveMovingObjects || (nobjects > 0);
    std::vector<Object*> objects =
        loadCSVMotionObjects(
            csvfile,
            std::tr1::bind(generatePosition, mRegion),
            nobjects
        );
    createCSVObjects(objects, nobjects);
}

void SimulatorBase::createCSVObjects(std::vector<Object*>& objects, int nobjects) {
    nobjects = std::min(nobjects, (int)objects.size()); // just in case we don't have enough

    // Update bounding box using full set of data
    for(int i = 0; i < (int)objects.size(); i++) {
        Object* obj = objects[i];
        mRegion.mergeIn( BoundingBox3(obj->position(Time::null()), obj->position(Time::null())) );
    }

    // Sample a subset of the objects
    for (int i = 0; i < nobjects; i++) {
        int x = rand() % objects.size();
        Object* obj = objects[x];
        objects.erase( objects.begin() + x );
        mAllObjects[obj->id()] = obj;
        if (obj->dynamic())
            mRemovedDynamicObjects[obj->id()] = obj;
        else
            mRemovedStaticObjects[obj->id()] = obj;
    }
    // Get rid of the leftovers
    for(int i = 0; i < (int)objects.size(); i++)
        delete objects[i];
}

void SimulatorBase::addObjects() {
    // All static, then all dynamic to get consistency across experiments that
    // compare mixed and unmixed trees.
    while(!mRemovedStaticObjects.empty()) {
        Object* obj = mRemovedStaticObjects.begin()->second;
        addObject(obj);
    }
    while(!mRemovedDynamicObjects.empty()) {
        Object* obj = mRemovedDynamicObjects.begin()->second;
        addObject(obj);
    }
}

void SimulatorBase::run() {
    addObjects();

    mTimer.start();
}

void SimulatorBase::shutdown() {
    // Remove all objects
    while(!mObjects.empty()) {
        Object* obj = mObjects.begin()->second;
        removeObject(obj);
    }
    // And delete them all
    while(!mRemovedDynamicObjects.empty()) {
        Object* obj = mRemovedDynamicObjects.begin()->second;
        delete obj;
        mRemovedDynamicObjects.erase(mRemovedDynamicObjects.begin());
    }
    while(!mRemovedStaticObjects.empty()) {
        Object* obj = mRemovedStaticObjects.begin()->second;
        delete obj;
        mRemovedStaticObjects.erase(mRemovedStaticObjects.begin());
    }

    delete mLocCache;
}

const BoundingBox3& SimulatorBase::region() const {
    return mRegion;
}

void SimulatorBase::addListener(SimulatorObjectListener* listener) {
    assert( std::find(mObjectListeners.begin(), mObjectListeners.end(), listener) == mObjectListeners.end() );
    mObjectListeners.push_back(listener);
}

void SimulatorBase::removeListener(SimulatorObjectListener* listener) {
    ObjectListenerList::iterator it = std::find(mObjectListeners.begin(), mObjectListeners.end(), listener);
    assert( it != mObjectListeners.end() );
    mObjectListeners.erase(it);
}

void SimulatorBase::tick() {
    Time last_time(mTime);
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

    tick_work(last_time, elapsed);
}

void SimulatorBase::tick_work(Time last_time, Duration elapsed) {
    //fprintf(stderr, "Tick: %f\n", (mTime - Time::null()).seconds());

    // Give all objects a chance to update their positions
    for(ObjectList::iterator it = mAllObjects.begin(); it != mObjects.end(); it++)
        it->second->tick(mTime);

    mIterations++;
    if (mTerminateIterations > 0 && mIterations >= mTerminateIterations)
        mFinished = true;
}

void SimulatorBase::addObject(Object* obj) {
    // Should find it in one of the two removed objects sets
    // Static
    OrderedObjectList::iterator oit = mRemovedStaticObjects.find(obj->id());
    if (oit != mRemovedStaticObjects.end()) mRemovedStaticObjects.erase(oit);
    // Dynamic
    oit = mRemovedDynamicObjects.find(obj->id());
    if (oit != mRemovedDynamicObjects.end()) mRemovedDynamicObjects.erase(oit);

    mObjects[obj->id()] = obj;
    mLocCache->addObject(obj);
    for(ObjectListenerList::iterator it = mObjectListeners.begin(); it != mObjectListeners.end(); it++)
        (*it)->simulatorAddedObject(obj, obj->position(), obj->bounds());
}

void SimulatorBase::removeObject(Object* obj) {
    ObjectList::iterator it = mObjects.find(obj->id());
    mObjects.erase(it);
    if (obj->dynamic())
        mRemovedDynamicObjects[obj->id()] = obj;
    else
        mRemovedStaticObjects[obj->id()] = obj;
    for(ObjectListenerList::iterator it = mObjectListeners.begin(); it != mObjectListeners.end(); it++)
        (*it)->simulatorRemovedObject(obj);
    mLocCache->removeObject(obj);
}

SimulatorBase::ObjectIterator SimulatorBase::objectsBegin() {
    return mObjects.begin();
}

SimulatorBase::ObjectIterator SimulatorBase::objectsEnd() {
    return mObjects.end();
}

SimulatorBase::ObjectIterator SimulatorBase::objectsFind(const ObjectID& objid) {
    return mObjects.find(objid);
}

int SimulatorBase::objectsSize() const {
    return mObjects.size();
}

int SimulatorBase::allObjectsSize() const {
    return mAllObjects.size();
}

} // namespace Simulation
} // namespace Prox
