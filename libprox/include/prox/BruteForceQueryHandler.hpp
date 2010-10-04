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

#include <prox/Constraints.hpp>

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
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef QueryCache<SimulationTraits> QueryCacheType;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;

    typedef typename QueryHandlerType::ShouldTrackCallback ShouldTrackCallback;

    BruteForceQueryHandler()
     : QueryHandlerType(),
       mLocCache(NULL)
    {
    }

    ~BruteForceQueryHandler() {
        for(ObjectSetIterator it = mObjects.begin(); it != mObjects.end(); it++) {
            mLocCache->stopTracking(it->second);
        }
        mObjects.clear();
        for(QueryMapIterator it = mQueries.begin(); it != mQueries.end(); it++) {
            QueryState* state = it->second;
            delete state;
        }
        mQueries.clear();

        mLocCache->removeUpdateListener(this);
    }

    void initialize(LocationServiceCacheType* loc_cache, bool static_objects, ShouldTrackCallback should_track_cb) {
        mLocCache = loc_cache;
        mLocCache->addUpdateListener(this);
        mShouldTrackCB = should_track_cb;
    }

    void tick(const Time& t) {
        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            QueryType* query = query_it->first;
            QueryState* state = query_it->second;
            QueryCacheType newcache;

            // Convert to a single world-space bounding sphere
            Vector3 query_pos = query->position(t);
            BoundingSphere query_region = query->region();
            float query_ms = query->maxSize();
            SolidAngle query_angle = query->angle();
            float query_radius = query->radius(); // cut off radius

            for(ObjectSetIterator obj_it = mObjects.begin(); obj_it != mObjects.end(); obj_it++) {
                ObjectID obj = obj_it->first;
                LocCacheIterator& obj_loc_it = obj_it->second;
                BoundingSphere obj_loc = mLocCache->worldRegion(obj_loc_it, t);
                float32 obj_size = mLocCache->maxSize(obj_loc_it);

                bool satisfies = satisfiesConstraintsBoundsAndMaxSize<SimulationTraits>(obj_loc.center(), obj_loc.radius(), obj_size, query_pos, query_region, query_ms, query_angle, query_radius);
                if (satisfies)
                    newcache.add(obj);
            }

            std::deque<QueryEventType> events;
            state->cache.exchange(newcache, &events);

            query->pushEvents(events);
        }
    }

    virtual uint32 numObjects() const {
        return (uint32)mObjects.size();
    }
    virtual uint32 numQueries() const {
        return (uint32)mQueries.size();
    }

    virtual LocationServiceCacheType* locationCache() const {
        return mLocCache;
    }

    void addObject(const ObjectID& obj_id) {
        mObjects[obj_id] = mLocCache->startTracking(obj_id);
    }

    void removeObject(const ObjectID& obj_id) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        LocCacheIterator obj_loc_it = it->second;
        mObjects.erase(it);
        mLocCache->stopTracking(obj_loc_it);
    }

    bool containsObject(const ObjectID& obj_id) {
        return (mObjects.find(obj_id) != mObjects.end());
    }

    void locationConnected(const ObjectID& obj_id, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        // Nothing to be done, we use values directly from the object
    }

    void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        // Nothing to be done, we use values directly from the object
    }

    void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) {
        // Nothing to be done, we use values directly from the object
    }

    void locationDisconnected(const ObjectID& obj_id) {
        removeObject(obj_id);
    }

    void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        // Nothing to be done, we use values directly from the query
    }

    void queryRegionChanged(QueryType* query, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
        // Nothing to be done, we use values directly from the query
    }

    void queryMaxSizeChanged(QueryType* query, Real old_maxSize, Real new_maxSize) {
        // Nothing to be done, we use values directly from the query
    }

    void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) {
        // Nothing to be done, we use values directly from the query
    }

    void queryDeleted(const QueryType* query) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;
        delete state;
        mQueries.erase(it);
    }

protected:
    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState;
        mQueries[query] = state;
        query->addChangeListener(this);
    }

private:
    struct QueryState {
        QueryCacheType cache;
    };

    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, ObjectIDHasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;

    LocationServiceCacheType* mLocCache;
    ShouldTrackCallback mShouldTrackCB;
    ObjectSet mObjects;
    QueryMap mQueries;
}; // class BruteForceQueryHandler

} // namespace Prox

#endif //_PROX_BRUTE_FORCE_QUERY_HANDLER_HPP_
