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

Simulator::Simulator(QueryHandler* handler)
 : mObjectIDSource(0),
   mHandler(handler),
   mLocCache(NULL)
{
}

Simulator::~Simulator() {
    // Remove all objects
    while(!mObjects.empty()) {
        Object* obj = mObjects.front();
        removeObject(obj);
    }
    // And delete them all
    while(!mRemovedObjects.empty()) {
        Object* obj = mRemovedObjects.front();
        delete obj;
    }

    while(!mQueries.empty()) {
        Query* query = mQueries.front();
        removeQuery(query);
        delete query;
    }

    delete mLocCache;
}

void Simulator::initialize(const Time& t, const BoundingBox3& region, int nobjects, int nqueries, int churnrate) {
    mChurn = churnrate;

    ObjectLocationServiceCache* loc_cache = new ObjectLocationServiceCache();
    addListener(loc_cache);
    mLocCache = loc_cache;

    mHandler->initialize(mLocCache);

    mRegion = region;
    Vector3 region_min = region.min();
    Vector3 region_extents = region.extents();

    // Generate objects
    for(int i = 0; i < nobjects; i++) {
        mObjectIDSource++;
        unsigned char oid_data[ObjectID::static_size]={0};
        memcpy(oid_data,&mObjectIDSource,ObjectID::static_size<sizeof(mObjectIDSource)?ObjectID::static_size:sizeof(mObjectIDSource));
        ObjectID oid(oid_data,ObjectID::static_size);

        Object* obj = new Object(
            ObjectID(oid_data,ObjectID::static_size),
            MotionVector3(
                t,
                region_min + Vector3(region_extents.x * randFloat(), region_extents.y * randFloat(), 0.f/*region_extents.z * randFloat()*/),
                Vector3(randFloat() * 20.f - 10.f, randFloat() * 20.f - 10.f, 0.f/*randFloat() * 20.f - 10.f*/)
            ),
            BoundingBox3( Vector3(-1, -1, -1), Vector3(1, 1, 1))
        );

        // Split objects into two groups. The first is added immediately,
        // guaranteeing testing of new queries over existing trees.  The rest
        // are left for churn, testing updates of queries as objects are added
        // and removed.

        // Always insert to mRemovedObjects first.  addObject will remove from mRemovedObjects.
        mRemovedObjects.push_back(obj);

        if (i > mChurn)
            addObject(obj);
    }

    // Queries
    for(int i = 0; i < nqueries; i++) {
        // Pick a random object to use as a basis for this query
        uint32 obj_idx = randUInt32(0, mObjects.size()-1);
        Object* obj = mObjects[obj_idx];

        Query* query = mHandler->registerQuery(
            obj->position(),
            BoundingSphere(obj->bounds().center(), 0),
            obj->bounds().radius(),
            SolidAngle( SolidAngle::Max / 1000 )
        );
        addQuery(query);
    }
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

void Simulator::tick(const Time& t) {
    for(int i = 0; !mObjects.empty() && i < mChurn; i++) {
        Object* obj = *(mObjects.begin());
        removeObject(obj);
    }
    for(int i = 0; !mRemovedObjects.empty() && i < mChurn; i++) {
        Object* obj = *(mRemovedObjects.begin());
        addObject(obj);
    }

    mHandler->tick(t);
}

void Simulator::addObject(Object* obj) {
    mRemovedObjects.erase( std::find(mRemovedObjects.begin(), mRemovedObjects.end(), obj) );
    mObjects.push_back(obj);
    mLocCache->addObject(obj);
    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorAddedObject(obj, obj->position(), obj->bounds());
}

void Simulator::removeObject(Object* obj) {
    ObjectList::iterator it = std::find(mObjects.begin(), mObjects.end(), obj);
    mObjects.erase(it);
    mRemovedObjects.push_back(obj);
    mLocCache->removeObject(obj);
    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorRemovedObject(obj);
}

void Simulator::addQuery(Query* query) {
    mQueries.push_back(query);
    for(ListenerList::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->simulatorAddedQuery(query);
}

void Simulator::removeQuery(Query* query) {
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

Simulator::QueryIterator Simulator::queriesBegin() {
    return mQueries.begin();
}

Simulator::QueryIterator Simulator::queriesEnd() {
    return mQueries.end();
}

} // namespace Simulation
} // namespace Prox
