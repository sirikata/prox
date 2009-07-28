/*  libprox
 *  BruteForceQueryHandler.hpp
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

#ifndef _PROX_BRUTE_FORCE_QUERY_HANDLER_HPP_
#define _PROX_BRUTE_FORCE_QUERY_HANDLER_HPP_

#include <prox/QueryHandler.hpp>
#include <prox/LocationUpdateListener.hpp>
#include <prox/QueryChangeListener.hpp>
#include <prox/QueryCache.hpp>

#include <prox/DefaultSimulationTraits.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class BruteForceQueryHandler : public QueryHandler<SimulationTraits> {
public:
    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef Query<SimulationTraits> QueryType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef QueryCache<SimulationTraits> QueryCacheType;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;


    BruteForceQueryHandler()
     : QueryHandlerType(),
       mLocCache(NULL)
    {
    }

    ~BruteForceQueryHandler() {
        for(ObjectSetIterator it = mObjects.begin(); it != mObjects.end(); it++) {
            mLocCache->stopTracking(*it);
        }
        mObjects.clear();
        for(QueryMapIterator it = mQueries.begin(); it != mQueries.end(); it++) {
            QueryState* state = it->second;
            delete state;
        }
        mQueries.clear();

        mLocCache->removeUpdateListener(this);
    }

    void initialize(LocationServiceCacheType* loc_cache) {
        mLocCache = loc_cache;
        mLocCache->addUpdateListener(this);
    }

    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState;
        mQueries[query] = state;
        query->addChangeListener(this);
    }

    void tick(const Time& t) {
        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            QueryType* query = query_it->first;
            QueryState* state = query_it->second;
            QueryCacheType newcache;

            for(ObjectSetIterator obj_it = mObjects.begin(); obj_it != mObjects.end(); obj_it++) {
                ObjectID obj = *obj_it;
                MotionVector3 obj_loc = mLocCache->location(obj);
                Vector3 obj_pos = obj_loc.position(t);
                BoundingSphere obj_bounds = mLocCache->bounds(obj);

                // Must satisfy radius constraint
                if (query->radius() != QueryType::InfiniteRadius && (obj_pos-query->position(t)).lengthSquared() > query->radius()*query->radius())
                    continue;

                // Must satisfy solid angle constraint
                Vector3 obj_world_center = obj_pos + obj_bounds.center();
                Vector3 to_obj = obj_world_center - query->position(t);
                SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, obj_bounds.radius());

                if (solid_angle < query->angle())
                    continue;

                newcache.add(obj);
            }

            std::deque<QueryEventType> events;
            state->cache.exchange(newcache, &events);

            query->pushEvents(events);
        }
    }

    void locationConnected(const ObjectID& obj_id, const MotionVector3& pos, const BoundingSphere& bounds) {
        mObjects.insert(obj_id);
        mLocCache->startTracking(obj_id);
    }

    void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        // Nothing to be done, we use values directly from the object
    }

    void locationBoundsUpdated(const ObjectID& obj_id, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
        // Nothing to be done, we use values directly from the object
    }

    void locationDisconnected(const ObjectID& obj_id) {
        assert( mObjects.find(obj_id) != mObjects.end() );
        mObjects.erase(obj_id);
        mLocCache->stopTracking(obj_id);
    }

    void queryPositionUpdated(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        // Nothing to be done, we use values directly from the query
    }

    void queryDeleted(const QueryType* query) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;
        delete state;
        mQueries.erase(it);
    }

private:
    struct QueryState {
        QueryCacheType cache;
    };

    typedef std::set<ObjectID> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;

    LocationServiceCacheType* mLocCache;
    ObjectSet mObjects;
    QueryMap mQueries;
}; // class BruteForceQueryHandler

} // namespace Prox

#endif //_PROX_BRUTE_FORCE_QUERY_HANDLER_HPP_
