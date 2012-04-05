// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_RTREE_MANUAL_QUERY_HANDLER_HPP_
#define _PROX_RTREE_MANUAL_QUERY_HANDLER_HPP_

#include <prox/manual/QueryHandler.hpp>
#include <prox/base/LocationUpdateListener.hpp>
#include <prox/base/QueryCache.hpp>

#include <prox/base/DefaultSimulationTraits.hpp>

#include <prox/rtree/RTree.hpp>
#include <prox/rtree/Cut.hpp>

#include <prox/manual/impl/RTreeManualNodeIterator.hpp>

namespace Prox {

/** RTreeManualQueryHandler uses an RTree data structure for objects and tracks
 *  each query with a cut. It is largely reactive because it is manually
 *  controlled -- upon request, cuts are moved up or down, or adjusted due to
 *  object additions and removals.
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class RTreeManualQueryHandler : public ManualQueryHandler<SimulationTraits> {
public:
    typedef SimulationTraits SimulationTraitsType;

    typedef ManualQueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef ManualQuery<SimulationTraits> QueryType;
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

    typedef typename std::tr1::function<RTreeManualQueryHandler*()> QueryHandlerCreator;

    typedef typename QueryHandlerType::NodeIterator NodeIterator;

    RTreeManualQueryHandler(uint16 elements_per_node)
     : QueryHandlerType(),
       mLocCache(NULL),
       mLocUpdateProvider(NULL),
       mRTree(NULL),
       mLastTime(Time::null()),
       mElementsPerNode(elements_per_node)
    {
    }

    virtual ~RTreeManualQueryHandler() {
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

        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;
        using std::tr1::placeholders::_3;

        mRTree = new RTree(
            mElementsPerNode, mLocCache, static_objects, /*report_restructures_cb*/0,
            this, QueryHandlerType::mAggregateListener,
            std::tr1::bind(&CutNode<SimulationTraits>::handleRootReplaced, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleSplit, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleLiftCut, _1, _2),
            std::tr1::bind(&Cut::handleReorderCut, _1, _2),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectInserted, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectRemoved, _1, _2, _3)
        );
        mRTree->initialize();
    }

    virtual bool staticOnly() const {
        return mRTree->staticObjects();
    }

    void tick(const Time& t, bool report) {
        mRTree->update(t);
        mRTree->verifyConstraints(t);

        // Currently, Cut and CutNode expect that *something* will be
        // periodically servicing the queries and will therefore take care of
        // pushing updates out. So for now, we do this here, although it would
        // be better if it was done only in response to events being added.
        for (QueryMapIterator qit = mQueries.begin(); qit != mQueries.end(); qit++) {
            qit->second->cut->pushEvents();
        }

        mLastTime = t;
    }

    virtual void rebuild() {
        // FIXME add rebuilding support
        // when adding rebuilding, make sure to change the rebuilding() method
        assert(false);
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

    void removeObject(const ObjectID& obj_id, bool temporary = false) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        LocCacheIterator obj_loc_it = it->second;
        deleteObj(obj_id, mLastTime, temporary);
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

    void locationDisconnected(const ObjectID& obj_id, bool permanent = false) {
        removeObject(obj_id, permanent);
    }

    // QueryChangeListener Implementation
    void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) {
    }

    void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) {
    }

    void queryMaxSizeChanged(QueryType* query, Real old_ms, Real new_ms) {
    }

    void queryDestroyed(QueryType* query, bool implicit) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;

        // Fill in removal events if they aren't implicit
        if (!implicit) {
            QueryEventType rem_evt;
            state->cut->destroyCut(rem_evt);
            query->pushEvent(rem_evt);
        }

        delete state;
        mQueries.erase(it);

        mRTree->verifyConstraints(mLastTime);
        //validateCuts();
    }

    void queryDeleted(const QueryType* query) {
    }


protected:
    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState(this, query, mRTree->root());
        mQueries[query] = state;
        query->addChangeListener(this);
    }

    bool refine(QueryType* query, const ObjectID& objid) {
        // If it's a leaf objects, we can't refine
        if (containsObject(objid)) return false;

        Cut* cut = mQueries[query]->cut;
        return cut->refine(objid);
    }

    bool coarsen(QueryType* query, const ObjectID& objid) {
        Cut* cut = mQueries[query]->cut;
        return cut->coarsen(objid);
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

    void deleteObj(const ObjectID& obj_id, const Time& t, bool temporary) {
        assert(mObjects.find(obj_id) != mObjects.end());
        mRTree->erase(mObjects[obj_id], t, temporary);
    }


    ///this needs to be a template class for no good reason: Microsoft visual studio bugs demand it.
    template <class XSimulationTraits> struct CutNode;
    class Cut;

public: // Public for the sake of implementation -- node iterators are separate classes
#ifndef LIBPROX_RTREE_DATA
# error "You must define LIBPROX_RTREE_DATA to either LIBPROX_RTREE_DATA_BOUNDS or LIBPROX_RTREE_DATA_MAXSIZE"
#endif
#if LIBPROX_RTREE_DATA == LIBPROX_RTREE_DATA_BOUNDS
    typedef BoundingSphereData<SimulationTraits, CutNode<SimulationTraits> > NodeData;
#elif LIBPROX_RTREE_DATA == LIBPROX_RTREE_DATA_MAXSIZE
    typedef MaxSphereData<SimulationTraits, CutNode<SimulationTraits> > NodeData;
#else
# error "Invalid setting for LIBPROX_RTREE_DATA"
#endif
    typedef Prox::RTree<SimulationTraits, NodeData, CutNode<SimulationTraits> > RTree;
protected:
    typedef typename RTree::RTreeNodeType RTreeNodeType;

    ///this needs to be a template class for no good reason: Microsoft visual studio bugs demand it.
    template <class XSimulationTraits> struct CutNode :
        public Prox::CutNodeBase<SimulationTraits, QueryHandlerType, NodeData, Cut, CutNode<SimulationTraits> >
    {
        typedef Prox::CutNodeBase<SimulationTraits, QueryHandlerType, NodeData, Cut, CutNode<SimulationTraits> > CutNodeBaseType;
        typedef typename CutNodeBaseType::CutType CutType;
        typedef typename CutNodeBaseType::RangeType RangeType;

        CutNode(QueryHandlerType* handler, Cut* _parent, RTreeNodeType* _rt, AggregateListenerType* listener)
         : CutNodeBaseType(handler, _parent, _rt, listener)
        {
        }

    private:
        friend class Prox::CutNodeBase<SimulationTraits, QueryHandlerType, NodeData, Cut, CutNode<SimulationTraits> >;
        ~CutNode() {
        }
    };

    friend class Prox::CutBase<SimulationTraits, RTreeManualQueryHandler, NodeData, Cut, CutNode<SimulationTraits> >;
    class Cut
        : public Prox::CutBase<SimulationTraits, RTreeManualQueryHandler, NodeData, Cut, CutNode<SimulationTraits> >
    {
        typedef Prox::CutBase<SimulationTraits, RTreeManualQueryHandler, NodeData, Cut, CutNode<SimulationTraits> > CutBaseType;
        typedef typename CutBaseType::CutNodeType CutNodeType;
        typedef typename CutBaseType::CutNodeList CutNodeList;
        typedef typename CutBaseType::CutNodeListIterator CutNodeListIterator;
        typedef typename CutBaseType::CutNodeListConstIterator CutNodeListConstIterator;

    private:
        Cut();

        using CutBaseType::parent;
        using CutBaseType::query;
        using CutBaseType::nodes;
        using CutBaseType::length;
        using CutBaseType::events;

        using CutBaseType::validateCut;

        typedef typename CutBaseType::ResultSet ResultSet;
        ResultSet results;


    public:

        /** Regular constructor.  A new cut simply starts with the root node and
         *  immediately refines.
         */
        Cut(RTreeManualQueryHandler* _parent, QueryType* _query, RTreeNodeType* root)
         : CutBaseType(_parent, _query)
        {
            init(root);
            pushEvents();
        }

        ~Cut() {
        }

        // Methods required by CutBase
        bool withAggregates() const {
            return true;
        }
        AggregateListenerType* aggregateListener() {
            return parent->aggregateListener();
        }
        LocationServiceCacheType* locCache() const {
            return parent->mLocCache;
        }
        const Time& curTime() const {
            return parent->mLastTime;
        }
        RTreeNodeType* rootRTreeNode() {
            return parent->mRTree->root();
        }
        bool rebuilding() const {
            return false;
        }
        void addResult(const ObjectID& objid) {
            results.insert(objid);
        }
        size_t removeResult(const ObjectID& objid) {
            return results.erase(objid);
        }
        bool inResults(const ObjectID& objid) const {
            return results.find(objid) != results.end();
        }
        int resultsSize() const {
            return results.size();
        }
        bool satisfiesQuery(RTreeNodeType* node, LocCacheIterator objit, int objidx) const {
            // This will be called when a node is inserted and this cut is
            // affected, i.e. when the node is inserted at a node this cut
            // crosses. For manually controlled, always indicate that the node
            // satisfies the query since we have no real test to perform. This
            // will ensure that if other children are being observed, this one
            // will be as well.
            return true;
        }

        // Push events if there are any queued up.
        void pushEvents() {
            if (!events.empty()) query->pushEvents(events);
        }

    public:

        // Returns the number of "nodes" visited, including objects.
        // In other words, gives the number of solid angle tests performed.
        int update(LocationServiceCacheType* loc, const Time& t) {
            return 0;
        }


        // These are methods specific to this type of cut processing
        CutNodeListIterator findCutNode(const ObjectID& objid) {
            // FIXME this is inefficient but we don't currently have a faster way of
            // looking up nodes/cut nodes
            CutNodeListIterator cut_node_it = nodes.begin();
            while(cut_node_it != nodes.end()) {
                // Not sure why, but we apparently have to dereference this in
                // the loop because it won't compile unless we assign it into a
                // variable.
                CutNode<SimulationTraits>* cnode = *cut_node_it;
                if (cnode->rtnode->aggregateID() == objid) break;
                cut_node_it++;
            }
            return cut_node_it;
        }

        bool refine(const ObjectID& objid) {
            // Check that it is refinable. It must be in the results still to be
            // refinable
            if (!isInResults(objid)) return false;

            // Find the cut node
            CutNodeListIterator cut_node_it = findCutNode(objid);
            if (cut_node_it == nodes.end()) return false;
            // and refine it
            CutNode<SimulationTraits>* cnode = *cut_node_it;
            if (cnode->rtnode->leaf()) {
                replaceParentWithChildrenResults(cnode);
            }
            else {
                QueryEventType evt;
                replaceParentWithChildren(cut_node_it, &evt);
                query->pushEvent(evt);
            }
            return true;
        }

        bool coarsen(const ObjectID& objid) {
            // Coarsening is essentially the same operation as lifting a
            // cut. We'll just reuse that logic, meaning we need to find a
            // CutNode to lift and the RTreeNode to lift up to. The CutNode can
            // be any node under the RTreeNode we want to lift to. The RTreeNode
            // is just going to be the parent of the node with the objid specified.

            // Find the associated CutNode, which might be out of date.
            CutNodeListIterator cut_node_it = findCutNode(objid);
            if (cut_node_it == nodes.end()) return false;

            // Get the parent RTreeNode
            CutNode<SimulationTraits>* from_cut_node = *cut_node_it;
            RTreeNodeType* to_rtree_node = from_cut_node->rtnode->parent();
            if (to_rtree_node == NULL) return false;

            // And just reuse the lift cut operation
            handleLiftCut(from_cut_node, to_rtree_node);

            pushEvents();

            return true;
        }

    };

    struct QueryState {
        QueryState(RTreeManualQueryHandler* _parent, QueryType* _query, RTreeNodeType* root)
        {
            cut = new Cut(_parent, _query, root);
        }

        ~QueryState() {
            delete cut;
        }

        Cut* cut;
    };

    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, ObjectIDHasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;

    AggregateListenerType* aggregateListener() {
        // Currently we only support cuts in the manual query handler -- its the
        // only efficient way to track and update query state.
        return QueryHandlerType::mAggregateListener;
    }


    typedef typename RTreeManualQueryHandlerImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorImpl;
    friend class RTreeManualQueryHandlerImpl::NodeIteratorImpl<SimulationTraits>;

    virtual NodeIteratorImpl* nodesBeginImpl() const {
        return new NodeIteratorImpl(mRTree->nodesBegin());
    }
    virtual NodeIteratorImpl* nodesEndImpl() const {
        return new NodeIteratorImpl(mRTree->nodesEnd());
    }


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

#endif //_PROX_RTREE_MANUAL_QUERY_HANDLER_HPP_
