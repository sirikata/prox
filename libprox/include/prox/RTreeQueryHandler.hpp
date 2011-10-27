/*  libprox
 *  RTreeQueryHandler.hpp
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

#ifndef _PROX_RTREE_QUERY_HANDLER_HPP_
#define _PROX_RTREE_QUERY_HANDLER_HPP_

#include <prox/QueryHandler.hpp>
#include <prox/LocationUpdateListener.hpp>
#include <prox/QueryChangeListener.hpp>
#include <prox/QueryCache.hpp>

#include <prox/DefaultSimulationTraits.hpp>

#include <prox/RTree.hpp>

namespace Prox {

/** RTreeQueryHandler is a base class for QueryHandlers that use an RTree
 *  without cuts and QueryCaches to resolve queries.  It provides a bunch of
 *  utility code, but no real query processing.
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class RTreeQueryHandler : public QueryHandler<SimulationTraits> {
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

    typedef typename std::tr1::function<RTreeQueryHandler*()> QueryHandlerCreator;

    RTreeQueryHandler(uint16 elements_per_node)
     : QueryHandlerType(),
       mLocCache(NULL),
       mLocUpdateProvider(NULL),
       mRTree(NULL),
       mLastTime(Time::null()),
       mElementsPerNode(elements_per_node)
    {
    }

    virtual ~RTreeQueryHandler() {
        destroyCurrentTree();

        for(QueryMapIterator it = mQueries.begin(); it != mQueries.end(); it++) {
            QueryState* state = it->second;
            delete state;
        }
        mQueries.clear();

        mLocUpdateProvider->removeUpdateListener(this);
    }

    void initialize(LocationServiceCacheType* loc_cache, LocationUpdateProviderType* loc_up_provider, bool static_objects, ShouldTrackCallback should_track_cb) {
        mLocCache = loc_cache;
        mLocUpdateProvider = loc_up_provider;
        mLocUpdateProvider->addUpdateListener(this);
        mShouldTrackCB = should_track_cb;

        mRTree = new RTree(
            mElementsPerNode, mLocCache, static_objects,
            std::tr1::bind(&RTreeQueryHandler::reportRestructures, this)
        );
    }

    void tick(const Time& t, bool report) {
        // Implementations should override this, but make sure to call preTick
        // and postTick
        preTick(t, report);
        postTick(t, report);
    }

    virtual void rebuild() {
        ObjectList objects = allObjects();
        bool static_objects = mRTree->staticObjects();

        // Destroy current tree
        destroyCurrentTree();

        // Copy objects into this list
        for(typename ObjectList::iterator it = objects.begin(); it != objects.end(); it++)
            mObjects[mLocCache->iteratorID(*it)] = *it;

        // Build new tree
        mRTree = new RTree(
            mElementsPerNode, mLocCache, static_objects,
            std::tr1::bind(&RTreeQueryHandler::reportRestructures, this)
        );
        mRTree->bulkLoad(objects, mLastTime);
    }

    virtual float cost() {
        return mRTree->cost(mLastTime);
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
        addObject(mLocCache->startTracking(obj_id));
    }
    void addObject(const LocCacheIterator& obj_loc_it) {
        ObjectID obj_id = mLocCache->iteratorID(obj_loc_it);
        assert(mObjects.find( obj_id) == mObjects.end());

        mObjects[obj_id] = obj_loc_it;
        insertObj(obj_id, mLastTime);

        // If the object had disconnected and reconnected, make sure we don't
        // mark it as permanently gone.
        typename ObjectIDSetType::iterator del_obj_it = mRemovedObjects.find(obj_id);
        if (del_obj_it != mRemovedObjects.end()) mRemovedObjects.erase(del_obj_it);
    }

    void removeObject(const ObjectID& obj_id) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        LocCacheIterator obj_loc_it = it->second;
        deleteObj(obj_id, mLastTime);
        mLocCache->stopTracking(obj_loc_it);
        mObjects.erase(it);
        mRemovedObjects.insert(obj_id);
    }

    bool containsObject(const ObjectID& obj_id) {
        return (mObjects.find(obj_id) != mObjects.end());
    }

    ObjectList allObjects() {
        ObjectList retval;
        retval.reserve(mObjects.size());
        for(typename ObjectSet::iterator it = mObjects.begin(); it != mObjects.end(); it++)
            retval.push_back(mLocCache->startTracking(it->first));
        return retval;
    }

    virtual void bulkLoad(const ObjectList& objects) {
        bool static_objects = mRTree->staticObjects();
        assert(mObjects.size() == 0);

        // Copy iterators into our storage
        for(typename ObjectList::const_iterator it = objects.begin(); it != objects.end(); it++)
            mObjects[mLocCache->iteratorID(*it)] = *it;

        mRTree->bulkLoad(objects, mLastTime);
    }


    void locationConnected(const ObjectID& obj_id, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        assert(mObjects.find(obj_id) == mObjects.end());

        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, local, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    // LocationUpdateListener Implementation
    void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationDisconnected(const ObjectID& obj_id) {
        removeObject(obj_id);
    }

    // QueryChangeListener Implementation
    void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        // Nothing to be done, we use values directly from the query
    }

    void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        // XXX FIXME
    }

    void queryMaxSizeChanged(QueryType* query, Real old_ms, Real new_ms) {
        // XXX FIXME
    }

    void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) {
        // XXX FIXME
    }

    void queryMaxResultsChanged(QueryType* query, const uint32 old_val, const uint32 new_val) {
        // Since we need to track removals properly (i.e. generate events) and
        // QueryCache won't do that if we set its size, we don't change the size
        // immediately. Instead, we'll just wait until the next iteration and
        // make future QueryCaches use the new size (still available in the
        // Query).
    }

    void queryDestroyed(QueryType* query, bool implicit) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;

        // Fill in removal events if they aren't implicit
        if (!implicit) {
            QueryCacheType emptycache(query->maxResults());
            std::deque<QueryEventType> events;
            state->cache.exchange(emptycache, &events, mRemovedObjects);
            query->pushEvents(events);
        }

        delete state;
        mQueries.erase(it);
    }

    void queryDeleted(const QueryType* query) {
    }

protected:
    void preTick(const Time& t, bool report) {
        mRTree->update(t);
        if (QueryHandlerType::mShouldRestructure)
            mRTree->restructure(t);

        mRTree->verifyConstraints(t);

        if ((QueryHandlerType::mTrackChecks || QueryHandlerType::mReportQueryStats) && report)
            printf("tick\n");
    }

    void postTick(const Time& t, bool report) {
        mLastTime = t;

        if (QueryHandlerType::mReportHealth && report) {
            QueryHandlerType::mItsSinceReportedHealth++;
            if (QueryHandlerType::mItsSinceReportedHealth >= QueryHandlerType::mReportHealthFrequency) {
                mRTree->reportBounds(t);
                QueryHandlerType::mItsSinceReportedHealth = 0;
            }
        }
        if (QueryHandlerType::mReportCost && report)
            printf("{ \"cost\" : %f }\n", cost());
    }

    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState(query->maxResults());
        mQueries[query] = state;
        query->addChangeListener(this);
    }


    void destroyCurrentTree() {
        delete mRTree;

        for(ObjectSetIterator it = mObjects.begin(); it != mObjects.end(); it++) {
            mLocCache->stopTracking(it->second);
        }
        mObjects.clear();
    }

    void insertObj(const ObjectID& obj_id, const Time& t) {
        mRTree->insert(mObjects[obj_id], t);
    }

    void updateObj(const ObjectID& obj_id, const Time& t) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        mRTree->update(mObjects[obj_id], t);
    }

    void deleteObj(const ObjectID& obj_id, const Time& t) {
        assert(mObjects.find(obj_id) != mObjects.end());
        mRTree->erase(mObjects[obj_id], t);
    }

    struct QueryState {
        QueryState(uint32 max_size)
         : cache(max_size)
        {}

        QueryCacheType cache;
    };

    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, ObjectIDHasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;

    struct Cut {
        void rebuildCutOrder() {}
    };
    struct CutNode {
        typedef Cut CutType;
    };

#if RTREE_DATA == RTREE_DATA_BOUNDS
    typedef BoundingSphereData<SimulationTraits, CutNode> NodeData;
#elif RTREE_DATA == RTREE_DATA_MAXSIZE
    typedef MaxSphereData<SimulationTraits, CutNode> NodeData;
#endif
    typedef Prox::RTree<SimulationTraits, NodeData, CutNode> RTree;
    typedef typename RTree::RTreeNodeType RTreeNodeType;

    LocationServiceCacheType* mLocCache;
    LocationUpdateProviderType* mLocUpdateProvider;
    ShouldTrackCallback mShouldTrackCB;

    RTree* mRTree;
    ObjectSet mObjects;
    ObjectIDSetType mRemovedObjects;
    QueryMap mQueries;
    Time mLastTime;
    uint8 mElementsPerNode;
}; // class RTreeQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_QUERY_HANDLER_HPP_
