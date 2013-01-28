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

#include <prox/geom/QueryHandler.hpp>
#include <prox/base/LocationUpdateListener.hpp>
#include <prox/geom/QueryChangeListener.hpp>
#include <prox/base/QueryCache.hpp>

#include <prox/base/DefaultSimulationTraits.hpp>

#include <prox/rtree/Constraints.hpp>

#include <prox/geom/impl/BruteForceNodeIterator.hpp>

namespace Prox {

template<typename SimulationTraits>
class BruteForceQueryHandler : public QueryHandler<SimulationTraits> {
public:
    typedef SimulationTraits SimulationTraitsType;

    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef Query<SimulationTraits> QueryType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef QueryCache<SimulationTraits> QueryCacheType;
    typedef typename QueryCacheType::ObjectIDSet ObjectIDSetType;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;

    typedef typename QueryHandlerType::ShouldTrackCallback ShouldTrackCallback;
    typedef typename QueryHandlerType::ObjectList ObjectList;

    typedef typename std::tr1::function<BruteForceQueryHandler*()> QueryHandlerCreator;

    static BruteForceQueryHandler* construct() {
        return new BruteForceQueryHandler();
    }
    static QueryHandlerCreator Constructor() {
        return std::tr1::bind(&BruteForceQueryHandler::construct);
    }

    BruteForceQueryHandler()
     : QueryHandlerType(),
       mLocCache(NULL),
       mLocUpdateProvider(NULL)
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

        mLocUpdateProvider->removeUpdateListener(this);
    }

    void initialize(LocationServiceCacheType* loc_cache, LocationUpdateProviderType* loc_up_provider, bool static_objects, bool replicated, ShouldTrackCallback should_track_cb = 0) {
        mLocCache = loc_cache;
        mLocUpdateProvider = loc_up_provider;
        mLocUpdateProvider->addUpdateListener(this);
        mShouldTrackCB = should_track_cb;
        mStaticObjects = static_objects;
    }

    virtual bool staticOnly() const {
        return mStaticObjects;
    }

    void tick(const Time& t, bool report) {
        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            QueryType* query = query_it->first;
            QueryState* state = query_it->second;
            QueryCacheType newcache(query->maxResults());

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

                float32 satisfies = satisfiesConstraintsBoundsAndMaxSize<SimulationTraits>(obj_loc.center(), obj_loc.radius(), obj_size, query_pos, query_region, query_ms, query_angle, query_radius);
                if (satisfies != -1)
                    newcache.add(obj, satisfies);
            }

            std::deque<QueryEventType> events;
            state->cache.exchange(QueryHandlerType::handlerID(), mLocCache, newcache, &events, mRemovedObjects);

            query->pushEvents(events);
        }
        // We can clear out permanently removed objects since we should have
        // processed all their updates already
        mRemovedObjects.clear();
    }

    virtual void rebuild() {
    }

    virtual float cost() {
        return numObjects();
    }

    virtual uint32 numObjects() const {
        return (uint32)mObjects.size();
    }
    virtual uint32 numQueries() const {
        return (uint32)mQueries.size();
    }
    virtual uint32 numNodes() const {
        return numObjects();
    }

    virtual ObjectID rootAggregateID()  {
        return typename SimulationTraits::ObjectIDNullType()();
    }

    virtual uint32 numResultsForQuery(const QueryType* q) const {
        QueryMapConstIterator it = mQueries.find(const_cast<QueryType*>(q));
        if (it == mQueries.end()) return 0; // For rebuilding query handler
        QueryState* state = it->second;
        return state->cache.size();
    }
    virtual uint32 sizeForQuery(const QueryType* q) const {
        // Same as numResults since it's just the set of objects we're tracking
        return numResultsForQuery(q);
    }

    virtual LocationServiceCacheType* locationCache() const {
        return mLocCache;
    }

    void addObject(const ObjectID& obj_id) {
        addObject(mLocCache->startTracking(obj_id));
    }
    void addObject(const ObjectID& obj_id, const ObjectID& parent_id) {
        addObject(mLocCache->startTracking(obj_id));
    }
    void addObject(const LocCacheIterator& obj_loc_it) {
        ObjectID obj_id = mLocCache->iteratorID(obj_loc_it);
        mObjects[obj_id] = obj_loc_it;
        // If the object had disconnected and reconnected, make sure we don't
        // mark it as permanently gone.
        typename ObjectIDSetType::iterator del_obj_it = mRemovedObjects.find(obj_id);
        if (del_obj_it != mRemovedObjects.end()) mRemovedObjects.erase(del_obj_it);
    }

    void removeObject(const ObjectID& obj_id, bool temporary = false) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        LocCacheIterator obj_loc_it = it->second;
        mObjects.erase(it);
        // Don't add to removed objects if it's temporary so that we generate
        // Transient removal events
        if (!temporary)
            mRemovedObjects.insert(obj_id);
        mLocCache->stopTracking(obj_loc_it);
    }

    bool containsObject(const ObjectID& obj_id) {
        return (mObjects.find(obj_id) != mObjects.end());
    }

    ObjectList allObjects() {
        ObjectList retval;
        retval.reserve(mObjects.size());
        for(typename ObjectSet::iterator it = mObjects.begin(); it != mObjects.end(); it++)
            retval.push_back( mLocCache->startTracking(it->first) );
        return retval;
    }

    void addNode(const ObjectID& objid, const ObjectID& parentid) {
        assert(false && "Brute force handler doesn't support replicating internal nodes.");
    }
    void removeNode(const ObjectID& objid, bool temporary = false) {
        assert(false && "Brute force handler doesn't support replicating internal nodes.");
    }

    void reparent(const ObjectID& objid, const ObjectID& parentid) {
        // Brute force doesn't need to care about reparenting since it just
        // holds a list -- parents aren't important.
    }

    void locationConnected(const ObjectID& obj_id, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, local, aggregate, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    void locationConnectedWithParent(const ObjectID& obj_id, const ObjectID& parent, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        // This query handler ignores parents
        locationConnected(obj_id, aggregate, local, pos, region, ms);
    }

    void locationParentUpdated(const ObjectID& obj_id, const ObjectID& old_par, const ObjectID& new_par) {
        // Nothing to be done, we use values directly from the object
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

    void locationQueryDataUpdated(const ObjectID& obj_id, const String& old_query_data, const String& new_query_data) {
        // Nothing to be done, we use values directly from the object
    }

    void locationDisconnected(const ObjectID& obj_id, bool temporary = false) {
        removeObject(obj_id, temporary);
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

    void queryMaxResultsChanged(QueryType* query, const uint32 old_val, const uint32 new_val) {
        // Since we need to track removals properly (i.e. generate events) and
        // QueryCache won't do that if we set its size, we don't change the size
        // immediately. Instead, we'll just wait until the next iteration and
        // make future QueryCaches use the new size (still available in the
        // Query).
    }

    void queryCustomQueryChanged(QueryType* query, const String& old_val, const String& new_val) {
        // No custom queries...
    }

    void queryDestroyed(QueryType* query, bool implicit) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;

        // Fill in removal events if they aren't implicit
        if (!implicit) {
            QueryCacheType emptycache(query->maxResults());
            std::deque<QueryEventType> events;
            state->cache.exchange(QueryHandlerType::handlerID(), mLocCache, emptycache, &events, mRemovedObjects);
            query->pushEvents(events);
        }

        // And clean up
        delete state;
        mQueries.erase(it);
    }

    void queryDeleted(const QueryType* query) {
    }

protected:
    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState(query->maxResults());
        mQueries[query] = state;
        query->addChangeListener(this);
    }

private:

    typedef typename BruteForceQueryHandlerImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorImpl;
    friend class BruteForceQueryHandlerImpl::NodeIteratorImpl<SimulationTraits>;

    virtual NodeIteratorImpl* nodesBeginImpl() const {
        return new NodeIteratorImpl(this, mObjects.begin());
    }
    virtual NodeIteratorImpl* nodesEndImpl() const {
        return new NodeIteratorImpl(this, mObjects.end());
    }


    struct QueryState {
        QueryState(uint32 max_size)
         : cache(max_size)
        {}

        QueryCacheType cache;
    };

    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, ObjectIDHasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef typename ObjectSet::const_iterator ObjectSetConstIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;
    typedef typename QueryMap::const_iterator QueryMapConstIterator;

    LocationServiceCacheType* mLocCache;
    LocationUpdateProviderType* mLocUpdateProvider;
    ShouldTrackCallback mShouldTrackCB;
    bool mStaticObjects;
    ObjectSet mObjects;
    ObjectIDSetType mRemovedObjects;
    QueryMap mQueries;
}; // class BruteForceQueryHandler

} // namespace Prox

#endif //_PROX_BRUTE_FORCE_QUERY_HANDLER_HPP_
