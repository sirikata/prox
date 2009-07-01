/*  libprox
 *  BruteForceQueryHandler.cpp
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

#include <prox/BruteForceQueryHandler.hpp>
#include <prox/BoundingSphere.hpp>
#include <cassert>

namespace Prox {

BruteForceQueryHandler::BruteForceQueryHandler()
 : QueryHandler(),
   LocationUpdateListener(),
   QueryChangeListener(),
   mLocCache(NULL)
{
}

BruteForceQueryHandler::~BruteForceQueryHandler() {
    for(ObjectSet::iterator it = mObjects.begin(); it != mObjects.end(); it++) {
        mLocCache->stopTracking(*it);
    }
    mObjects.clear();
    for(QueryMap::iterator it = mQueries.begin(); it != mQueries.end(); it++) {
        QueryState* state = it->second;
        delete state;
    }
    mQueries.clear();

    mLocCache->removeUpdateListener(this);
}

void BruteForceQueryHandler::initialize(LocationServiceCache* loc_cache) {
    mLocCache = loc_cache;
    mLocCache->addUpdateListener(this);
}

void BruteForceQueryHandler::registerObject(const ObjectID& obj_id) {
    mObjects.insert(obj_id);
    mLocCache->startTracking(obj_id);
}

void BruteForceQueryHandler::registerQuery(Query* query) {
    QueryState* state = new QueryState;
    mQueries[query] = state;
    query->addChangeListener(this);
}

void BruteForceQueryHandler::tick(const Time& t) {
    for(QueryMap::iterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
        Query* query = query_it->first;
        QueryState* state = query_it->second;
        QueryCache newcache;

        for(ObjectSet::iterator obj_it = mObjects.begin(); obj_it != mObjects.end(); obj_it++) {
            ObjectID obj = *obj_it;
            MotionVector3f obj_loc = mLocCache->location(obj);
            Vector3f obj_pos = obj_loc.position(t);
            BoundingSphere3f obj_bounds = mLocCache->bounds(obj);

            // Must satisfy radius constraint
            if (query->radius() != Query::InfiniteRadius && (obj_pos-query->position(t)).lengthSquared() > query->radius()*query->radius())
                continue;

            // Must satisfy solid angle constraint
            Vector3f obj_world_center = obj_pos + obj_bounds.center();
            Vector3f to_obj = obj_world_center - query->position(t);
            SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, obj_bounds.radius());

            if (solid_angle < query->angle())
                continue;

            newcache.add(obj);
        }

        std::deque<QueryEvent> events;
        state->cache.exchange(newcache, &events);

        query->pushEvents(events);
    }
}

void BruteForceQueryHandler::locationPositionUpdated(const ObjectID& obj_id, const MotionVector3f& old_pos, const MotionVector3f& new_pos) {
    // Nothing to be done, we use values directly from the object
}

void BruteForceQueryHandler::locationBoundsUpdated(const ObjectID& obj_id, const BoundingSphere3f& old_bounds, const BoundingSphere3f& new_bounds) {
    // Nothing to be done, we use values directly from the object
}

void BruteForceQueryHandler::locationDisconnected(const ObjectID& obj_id) {
    assert( mObjects.find(obj_id) != mObjects.end() );
    mObjects.erase(obj_id);
    mLocCache->stopTracking(obj_id);
}

void BruteForceQueryHandler::queryPositionUpdated(Query* query, const MotionVector3f& old_pos, const MotionVector3f& new_pos) {
    // Nothing to be done, we use values directly from the query
}

void BruteForceQueryHandler::queryDeleted(const Query* query) {
    QueryMap::iterator it = mQueries.find(const_cast<Query*>(query));
    assert( it != mQueries.end() );
    QueryState* state = it->second;
    delete state;
    mQueries.erase(it);
}

} // namespace Prox
