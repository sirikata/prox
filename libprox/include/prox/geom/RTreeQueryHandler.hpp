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

#include <prox/geom/QueryHandler.hpp>
#include <prox/base/LocationUpdateListener.hpp>
#include <prox/geom/QueryChangeListener.hpp>
#include <prox/base/QueryCache.hpp>

#include <prox/base/DefaultSimulationTraits.hpp>

#include <prox/rtree/RTree.hpp>

#include <prox/geom/impl/RTreeNodeIterator.hpp>

namespace Prox {

/** RTreeQueryHandler is a base class for QueryHandlers that use an RTree
 *  without cuts and QueryCaches to resolve queries.  It provides a bunch of
 *  utility code, but no real query processing.
 */
template<typename SimulationTraits, typename NodeDataType>
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

    typedef typename QueryHandlerType::NodeIterator NodeIterator;

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

    void initialize(LocationServiceCacheType* loc_cache, LocationUpdateProviderType* loc_up_provider, bool static_objects, bool replicated, ShouldTrackCallback should_track_cb = 0) {
        mLocCache = loc_cache;
        mLocUpdateProvider = loc_up_provider;
        mLocUpdateProvider->addUpdateListener(this);
        mShouldTrackCB = should_track_cb;

        mRTree = new RTree(
            mElementsPerNode, mLocCache, static_objects, replicated,
            std::tr1::bind(&RTreeQueryHandler::reportRestructures, this),
            std::tr1::bind(&RTreeQueryHandler::handleRootCreated, this)
        );
    }

    virtual bool staticOnly() const {
        return mRTree->staticObjects();
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
        bool replicated = mRTree->replicated();

        // Destroy current tree
        destroyCurrentTree();

        // Copy objects into this list
        for(typename ObjectList::iterator it = objects.begin(); it != objects.end(); it++)
            mObjects[mLocCache->iteratorID(*it)] = *it;

        // Build new tree
        mRTree = new RTree(
            mElementsPerNode, mLocCache, static_objects, replicated,
            std::tr1::bind(&RTreeQueryHandler::reportRestructures, this),
            std::tr1::bind(&RTreeQueryHandler::handleRootCreated, this)
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
    virtual uint32 numNodes() const {
        // This is inefficient, but the RTree doesn't track number of nodes
        // internally.
        uint32 count = 0;
        NodeIterator it = QueryHandlerType::nodesBegin();
        NodeIterator end_it = QueryHandlerType::nodesEnd();
        while(it != end_it) {
            count++; it++;
        }
        return count;
    }

    virtual ObjectID rootAggregateID()  {
        return (mRTree->root() ? mRTree->root()->aggregateID() : typename SimulationTraits::ObjectIDNullType()());
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

    virtual void draw() { mRTree->draw(); }

    void addObject(const ObjectID& obj_id) {
        addObject(mLocCache->startTracking(obj_id));
    }
    void addObject(const ObjectID& obj_id, const ObjectID& parent_id) {
        addObject(mLocCache->startTracking(obj_id), parent_id);
    }
    void addObject(const LocCacheIterator& obj_loc_it, const ObjectID& parent_id) {
        ObjectID obj_id = mLocCache->iteratorID(obj_loc_it);
        assert(mObjects.find( obj_id) == mObjects.end());

        mObjects[obj_id] = obj_loc_it;
        mRTree->insert(mObjects[obj_id], mLastTime);

        // If the object had disconnected and reconnected, make sure we don't
        // mark it as permanently gone.
        typename ObjectIDSetType::iterator del_obj_it = mRemovedObjects.find(obj_id);
        if (del_obj_it != mRemovedObjects.end()) mRemovedObjects.erase(del_obj_it);
    }
    void addObject(const LocCacheIterator& obj_loc_it) {
        ObjectID obj_id = mLocCache->iteratorID(obj_loc_it);
        assert(mObjects.find( obj_id) == mObjects.end());

        mObjects[obj_id] = obj_loc_it;
        mRTree->insert(mObjects[obj_id], mLastTime);

        // If the object had disconnected and reconnected, make sure we don't
        // mark it as permanently gone.
        typename ObjectIDSetType::iterator del_obj_it = mRemovedObjects.find(obj_id);
        if (del_obj_it != mRemovedObjects.end()) mRemovedObjects.erase(del_obj_it);
    }

    void removeObject(const ObjectID& obj_id, bool temporary = false) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        LocCacheIterator obj_loc_it = it->second;
        deleteObj(obj_id, mLastTime, temporary);
        mLocCache->stopTracking(obj_loc_it);
        mObjects.erase(it);
        if (!temporary)
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

    void addNode(const ObjectID& nodeid, const ObjectID& parent) {
        addNode(mLocCache->startTracking(nodeid), parent);
    }
    void addNode(const LocCacheIterator& node_loc_it, const ObjectID& parent) {
        ObjectID node_id = mLocCache->iteratorID(node_loc_it);
        assert(mNodes.find(node_id) == mNodes.end());

        mNodes[node_id] = node_loc_it;
        mRTree->insertNode(node_loc_it, parent, mLastTime);

        // If the object had disconnected and reconnected, make sure we don't
        // mark it as permanently gone.
        typename ObjectIDSetType::iterator del_node_it = mRemovedObjects.find(node_id);
        if (del_node_it != mRemovedObjects.end()) mRemovedObjects.erase(del_node_it);
    }
    void removeNode(const ObjectID& nodeid, bool temporary = false) {
        typename ObjectSet::iterator it = mNodes.find(nodeid);
        if (it == mNodes.end()) return;

        LocCacheIterator node_loc_it = it->second;
        mRTree->eraseNode(node_loc_it, mLastTime, temporary);
        mLocCache->stopTracking(node_loc_it);
        mNodes.erase(it);
        mRemovedNodes.insert(nodeid);
    }


    void reparent(const ObjectID& nodeobjid, const ObjectID& parentid) {
        typename ObjectSet::iterator obj_it = mObjects.find(nodeobjid);
        if (obj_it != mObjects.end()) {
            mRTree->reparentObject(obj_it->second, parentid, mLastTime);
            return;
        }

        typename ObjectSet::iterator node_it = mNodes.find(nodeobjid);
        if (node_it != mNodes.end()) {
            mRTree->reparentNode(node_it->second, parentid, mLastTime);
            return;
        }
    }


    virtual void bulkLoad(const ObjectList& objects) {
        bool static_objects = mRTree->staticObjects();
        assert(mObjects.size() == 0);

        // Copy iterators into our storage
        for(typename ObjectList::const_iterator it = objects.begin(); it != objects.end(); it++)
            mObjects[mLocCache->iteratorID(*it)] = *it;

        mRTree->bulkLoad(objects, mLastTime);
    }


    void locationConnected(const ObjectID& obj_id, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        assert(mObjects.find(obj_id) == mObjects.end());

        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, local, aggregate, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    void locationConnectedWithParent(const ObjectID& obj_id, const ObjectID& parent, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        assert(mObjects.find(obj_id) == mObjects.end());

        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, local, aggregate, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    // LocationUpdateListener Implementation
    void locationParentUpdated(const ObjectID& obj_id, const ObjectID& old_par, const ObjectID& new_par) {
        reparent(obj_id, new_par);
    }

    void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationQueryDataUpdated(const ObjectID& obj_id, const String& old_query_data, const String& new_query_data) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationDisconnected(const ObjectID& obj_id, bool temporary = false) {
        removeObject(obj_id, temporary);
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

    void updateObj(const ObjectID& obj_id, const Time& t) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it != mObjects.end()) {
            mRTree->update(mObjects[obj_id], t);
        }

        typename ObjectSet::iterator nit = mNodes.find(obj_id);
        if (nit != mNodes.end()) {
            mRTree->updateNode(mNodes[obj_id], t);
        }
    }

    void deleteObj(const ObjectID& obj_id, const Time& t, bool temporary) {
        assert(mObjects.find(obj_id) != mObjects.end());
        mRTree->erase(mObjects[obj_id], t, temporary);
    }


    void handleRootCreated() {
        // We don't care about this because we just start from the root and
        // reevaluate each time. Not having a root just means we didn't have any
        // results before.
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
    typedef typename QueryMap::const_iterator QueryMapConstIterator;

    struct Cut {
        void rebuildCutOrder() {}
    };
    struct CutNode {
        typedef Cut CutType;
        typedef std::pair<CutNode*, CutNode*> RangeType;

        Cut* getParent() const { return (Cut*)0xdeadbeef; }
    };

public: // Public for the sake of implementation -- node iterators are separate classes
    typedef NodeDataType NodeData;
    typedef Prox::RTree<SimulationTraits, NodeData, CutNode> RTree;
protected:
    typedef typename RTreeQueryHandlerImpl::NodeIteratorImpl<SimulationTraits, NodeData> NodeIteratorImpl;
    friend class RTreeQueryHandlerImpl::NodeIteratorImpl<SimulationTraits, NodeData>;

    virtual NodeIteratorImpl* nodesBeginImpl() const {
        return new NodeIteratorImpl(mRTree->nodesBegin());
    }
    virtual NodeIteratorImpl* nodesEndImpl() const {
        return new NodeIteratorImpl(mRTree->nodesEnd());
    }

protected:
    typedef typename RTree::RTreeNodeType RTreeNodeType;

    LocationServiceCacheType* mLocCache;
    LocationUpdateProviderType* mLocUpdateProvider;
    ShouldTrackCallback mShouldTrackCB;

    RTree* mRTree;
    ObjectSet mObjects;
    ObjectIDSetType mRemovedObjects;
    ObjectSet mNodes;
    ObjectIDSetType mRemovedNodes;
    QueryMap mQueries;
    Time mLastTime;
    uint8 mElementsPerNode;
}; // class RTreeQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_QUERY_HANDLER_HPP_
