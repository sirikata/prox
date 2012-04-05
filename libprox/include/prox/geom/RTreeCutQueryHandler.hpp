/*  libprox
 *  RTreeCutQueryHandler.hpp
 *
 *  Copyright (c) 2010, Ewen Cheslack-Postava
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

#ifndef _PROX_RTREE_CUT_QUERY_HANDLER_HPP_
#define _PROX_RTREE_CUT_QUERY_HANDLER_HPP_

#include <prox/geom/QueryHandler.hpp>
#include <prox/base/LocationUpdateListener.hpp>
#include <prox/geom/QueryChangeListener.hpp>
#include <prox/base/QueryCache.hpp>

#include <prox/base/DefaultSimulationTraits.hpp>

#include <prox/rtree/RTree.hpp>
#include <prox/rtree/Cut.hpp>

#include <prox/geom/impl/RTreeCutNodeIterator.hpp>

namespace Prox {

/** Implementation of QueryHandler which uses cuts through an RTree to track the
 *  active set of objects/nodes for a query.
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class RTreeCutQueryHandler : public QueryHandler<SimulationTraits> {
public:
    typedef SimulationTraits SimulationTraitsType;

    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef Query<SimulationTraits> QueryType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

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

    typedef typename std::tr1::function<RTreeCutQueryHandler*()> QueryHandlerCreator;

    typedef typename QueryHandlerType::NodeIterator NodeIterator;

    static RTreeCutQueryHandler* construct(uint16 elements_per_node, bool with_aggregates) {
        return new RTreeCutQueryHandler(elements_per_node, with_aggregates);
    }
    static QueryHandlerCreator Constructor(uint16 elements_per_node, bool with_aggregates) {
        return std::tr1::bind(&RTreeCutQueryHandler::construct, elements_per_node, with_aggregates);
    }

    RTreeCutQueryHandler(uint16 elements_per_node, bool with_aggregates)
     : QueryHandlerType(),
       mLocCache(NULL),
       mLocUpdateProvider(NULL),
       mRTree(NULL),
       mLastTime(Time::null()),
       mElementsPerNode(elements_per_node),
       mWithAggregates(with_aggregates),
       mRebuilding(false)
    {
    }

    virtual ~RTreeCutQueryHandler() {
        for(QueryMapIterator it = mQueries.begin(); it != mQueries.end(); it++) {
            QueryState* state = it->second;
            delete state;
        }
        mQueries.clear();

        destroyCurrentTree();

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
            mElementsPerNode, mLocCache,
            static_objects,
            std::tr1::bind(&RTreeCutQueryHandler::reportRestructures, this),
            this, aggregateListener(),
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

    void validateCuts() const {
#ifdef PROXDEBUG
        for(QueryMapConstIterator it = mQueries.begin(); it != mQueries.end(); it++) {
            QueryState* state = it->second;
            state->cut->validateCut();
        }
#endif
    }

    void tick(const Time& t, bool report) {
        mRTree->update(t);
        if (QueryHandlerType::mShouldRestructure)
            mRTree->restructure(t);

        mRTree->verifyConstraints(t);
        validateCuts();

        if (QueryHandlerType::mReportQueryStats && report)
            printf("tick\n");

        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            QueryType* query = query_it->first;
            QueryState* state = query_it->second;

            int visited = state->cut->update(mLocCache, t);

            if (QueryHandlerType::mReportQueryStats && report) {
                printf("{ \"id\" : %d, \"checks\" : %d, \"cut-length\" : %d, \"results\" : %d }\n", query->id(), visited, state->cut->cutSize(), state->cut->resultsSize());
            }
        }
        mLastTime = t;

        mRTree->verifyConstraints(t);
        validateCuts();

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

    virtual void rebuild() {
        validateCuts();

        // First, get all cuts out of the original tree
        for(typename QueryMap::iterator it = mQueries.begin(); it != mQueries.end(); it++)
            it->second->cut->startSwapTrees();

        // Then rebuild
        mRebuilding = true;

        ObjectList objects = allObjects();
        bool static_objects = mRTree->staticObjects();

        // Destroy current tree
        destroyCurrentTree();

        // Build new tree
        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;
        using std::tr1::placeholders::_3;
        mRTree = new RTree(
            mElementsPerNode, mLocCache,
            static_objects,
            std::tr1::bind(&RTreeCutQueryHandler::reportRestructures, this),
            this, aggregateListener(),
            std::tr1::bind(&CutNode<SimulationTraits>::handleRootReplaced, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleSplit, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleLiftCut, _1, _2),
            std::tr1::bind(&Cut::handleReorderCut, _1, _2),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectInserted, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectRemoved, _1, _2, _3)
        );
        mRTree->initialize();
        mRTree->bulkLoad(objects, mLastTime);

        mRebuilding = false;

        // Then reinsert into the new tree
        for(typename QueryMap::iterator it = mQueries.begin(); it != mQueries.end(); it++)
            it->second->cut->finishSwapTrees(mRTree->root());

        validateCuts();
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
        assert(mObjects.find(obj_id) == mObjects.end());

        mObjects[obj_id] = obj_loc_it;
        insertObj(obj_id, mLastTime);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
    }

    void removeObject(const ObjectID& obj_id, bool temporary = false) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        mRTree->verifyConstraints(mLastTime);
        validateCuts();

        LocCacheIterator obj_loc_it = it->second;
        deleteObj(obj_id, mLastTime, temporary);
        mLocCache->stopTracking(obj_loc_it);
        mObjects.erase(it);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
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


    void locationConnected(const ObjectID& obj_id, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        assert(mObjects.find(obj_id) == mObjects.end());

        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, local, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    void locationConnectedWithParent(const ObjectID& obj_id, const ObjectID& parent, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        assert(mObjects.find(obj_id) == mObjects.end());

        bool do_track = true;
        if (mShouldTrackCB) do_track = mShouldTrackCB(obj_id, local, pos, region, ms);

        if (do_track)
            addObject(obj_id);
    }

    // LocationUpdateListener Implementation
    void locationParentUpdated(const ObjectID& obj_id, const ObjectID& old_par, const ObjectID& new_par) {
    }

    void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        updateObj(obj_id, mLastTime); // FIXME new time?

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
    }

    void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        updateObj(obj_id, mLastTime); // FIXME new time?

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
    }

    void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) {
        updateObj(obj_id, mLastTime); // FIXME new time?

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
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
        // XXX FIXME
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
        validateCuts();
    }

    void queryDeleted(const QueryType* query) {
    }

protected:
    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState(this, query, mRTree->root());
        mQueries[query] = state;
        query->addChangeListener(this);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
    }

private:
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
    struct Cut;

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
private:
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

    friend class Prox::CutBase<SimulationTraits, RTreeCutQueryHandler, NodeData, Cut, CutNode<SimulationTraits> >;
    class Cut
        : public Prox::CutBase<SimulationTraits, RTreeCutQueryHandler, NodeData, Cut, CutNode<SimulationTraits> >
    {
    private:
        Cut();

        typedef Prox::CutBase<SimulationTraits, RTreeCutQueryHandler, NodeData, Cut, CutNode<SimulationTraits> > CutBaseType;
        typedef typename CutBaseType::CutNodeType CutNodeType;
        typedef typename CutBaseType::CutNodeList CutNodeList;
        typedef typename CutBaseType::CutNodeListIterator CutNodeListIterator;
        typedef typename CutBaseType::CutNodeListConstIterator CutNodeListConstIterator;

        using CutBaseType::parent;
        using CutBaseType::query;
        using CutBaseType::nodes;
        using CutBaseType::length;
        using CutBaseType::events;
    public:
        using CutBaseType::validateCut;
    private:
        typedef typename CutBaseType::ResultSet ResultSet;
        ResultSet results;


    public:

        /** Regular constructor.  A new cut simply starts with the root node and
         *  immediately refines.
         */
        Cut(RTreeCutQueryHandler* _parent, QueryType* _query, RTreeNodeType* root)
         : CutBaseType(_parent, _query)
        {
            init(root);
            // Make sure we get events from initialization to the client
            if (!events.empty()) query->pushEvents(events);
        }

        ~Cut() {
        }

        // Methods required by CutBase
        bool withAggregates() const {
            return parent->mWithAggregates;
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
            return parent->mRebuilding;
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
            Time t = curTime();
            Vector3 qpos = query->position(t);
            BoundingSphere qregion = query->region();
            float qmaxsize = query->maxSize();
            const SolidAngle& qangle = query->angle();
            float qradius = query->radius();

            ObjectID child_id = this->getLocCache()->iteratorID(objit);
            bool child_satisfies = node->childData(objidx, this->getLocCache(), t).satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
            return child_satisfies;
        }
    private:
        // Struct which keeps track of how many children do not satisfy the
        // constraint so they can be collapsed.
        struct ParentCollapseInfo {
            RTreeNodeType* parent;
            int count;
        };

    public:
        // Checks for child_id's membership in the result set.  This version
        // should be used for non-aggregate queries.
        void checkMembership(const ObjectID& child_id, const NodeData& child_data, const Vector3& qpos, const BoundingSphere& qregion, float qmaxsize, const SolidAngle& qangle, float qradius) {
            bool child_satisfies = child_data.satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
            updateMembership(child_id, child_satisfies);
        }

        // Returns the number of "nodes" visited, including objects.
        // In other words, gives the number of solid angle tests performed.
        int update(LocationServiceCacheType* loc, const Time& t) {
            // Update assumes that the cut is already valid, i.e. that any
            // adjustments to the tree have already caused fixes in the cut
            // itself. The only thing that should have changed at this point is
            // the query itself. Therefore, we can simply expand the cut down
            // and pull it up as appropriate.
            //
            // We traverse the cut linearly and handle a few cases based on the
            // rule that we store cut nodes at RTree nodes if they are leaf
            // nodes (that satisfy or do not satisfy the solid angle constraint)
            // or at internal nodes which do not satisfy the solid angle
            // constraint.
            //
            // 1. If we encounter an internal node that satisfies the
            // constraint, we expand that node to contain all child nodes.  We
            // then continue iteration at the first of those new nodes.
            //
            // 2. If we encounter a leaf node, we test the true leaf children
            // iff the leaf node satisfies the constraints.  When testing true
            // leaves we just ensure they are appropriately in or out of the
            // result set.
            //
            // 3. (Continued from updateDown) If all the children in a parent
            // node are not satisfying the constraint, check if the parent
            // does. If the parent also does not, push the cut up to the parent
            // node.  Repeat until no more "merging" or pushing the cut up can
            // occur.
            // NOTE: 3 is not implemented. Doing this would make everything more
            // efficient but shouldn't affect correctness.

            // FIXME we should handle a few different cases differently here.
            // If the query is new, has been updated, or is moving, we should
            // evaluate across the entire tree.  If none of these things have
            // happened, previous updating should have taken care of everything
            // except pushing the cut back up the tree for now-unsatisfied
            // nodes.

            int visited = 0;

            // Keeps track of candidates for collapse and the number of children
            // they have that don't satisfy the constraint. Form a stack since
            // they may get deeper in the tree.
            std::stack<ParentCollapseInfo> collapseStack;

            Vector3 qpos = query->position(t);
            BoundingSphere qregion = query->region();
            float qmaxsize = query->maxSize();
            const SolidAngle& qangle = query->angle();
            float qradius = query->radius();

            for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); ) {
                CutNode<SimulationTraits>* node = *it;
                bool last_satisfies = node->satisfies;
                bool satisfies = node->updateSatisfies(qpos, qregion, qmaxsize, qangle, qradius);
                visited++;

                // If we went from satisfies -> not satisfies, we may need to
                // clean up some results
                if (last_satisfies && !satisfies) {
                    // If we're tracking aggregates and we went from satisfies ->
                    // not satisfies, then we need to clear out any children
                    if (parent->mWithAggregates) {
                        typename ResultSet::iterator this_result_it = results.find(node->rtnode->aggregateID());
                        if (this_result_it != results.end()) {
                            // Either this node was in the result set last time...
                            // And there's nothing to do -- it will be removed when
                            // the cut is pushed up the tree
                        }
                        else {
                            // Or its children were, in which case we remove them and
                            // add this node (even though its not a real result).
                            // If this change allows merging to the parent node of
                            // this node, that'll happen upon push-up
                            QueryEventType evt;
                            evt.additions().push_back( typename QueryEventType::Addition(node->rtnode, QueryEventType::Imposter) );
                            results.insert( node->rtnode->aggregateID() );
                            for(int i = 0; i < node->rtnode->size(); i++) {
                                ObjectID child_id = loc->iteratorID(node->rtnode->object(i).object);
                                typename ResultSet::iterator result_it = results.find(child_id);
                                assert(result_it != results.end());
                                results.erase(result_it);
                                evt.removals().push_back( typename QueryEventType::Removal(child_id, QueryEventType::Transient) );
                            }
                            events.push_back(evt);
                        }
                    }
                    else {
                        // With no aggregates, if this is a leaf, we need to
                        // clear out the children
                        if (node->leaf()) {
                            for(int i = 0; i < node->rtnode->size(); i++) {
                                ObjectID child_id = loc->iteratorID(node->rtnode->object(i).object);
                                checkMembership(child_id, node->rtnode->childData(i, loc, t), qpos, qregion, qmaxsize, qangle, qradius);
                                visited++;
                            }
                        }
                    }
                }

                // Aside from clean up above, if we don't satisfy the
                // constraints there's no reason to continue with this node.
                if (!satisfies) {
                    // This section deals with tracking sequences of nodes that
                    // do not satisfy the constraint.  The basic approach is to
                    // maintain a stack of valid ancestors that have been
                    // encountered on the cut, keep a count of children that do
                    // not satisfy them, and collapse as appropriate.  The stack
                    // is necessary because a cut might start high in the tree
                    // and then get deeper; the deeper part could collapse,
                    // leaving the earlier, higher part also collapsable. We
                    // need the stack to keep track of these other candidates.
                    // The stack gets cleaned out as we discover that the parent
                    // node currently being considered is not an ancestor of the
                    // current cut node anymore.

                    RTreeNodeType* cur_parent = (collapseStack.empty() ? NULL : collapseStack.top().parent);

                    // First make sure we have the right parent.
                    RTreeNodeType* this_node = node->rtnode;
                    RTreeNodeType* this_parent = node->rtnode->parent();
                    // If we have a mismatch:
                    if (this_parent != cur_parent) {
                        // First we check if we need to pop nodes off since we
                        // may have left a subtree.
                        while(!collapseStack.empty()) {
                            // FIXME this could be more efficient by tracing up
                            // the parent tree and the stack at the same time.
                            if (!_is_ancestor(this_node, collapseStack.top().parent))
                                collapseStack.pop();
                            else
                                break;
                        }
                        // Update cur_parent to reflect new state
                        cur_parent = (collapseStack.empty() ? NULL : collapseStack.top().parent);

                        // Then we recheck if we still have a mismatch --
                        // popping nodes may have found us the parent we were
                        // looking for.  If we do have a mismatch, we need to
                        // pop a new node on, but only if it is worth it (this
                        // node is the first child node of the parent, otherwise
                        // we should have found the parent or the first node
                        // satisfied the constraint and this parent isn't really
                        // an option).
                        if (this_parent != cur_parent) {
                            if (this_parent != NULL &&
                                this_parent->node(0) == node->rtnode)
                            {
                                ParentCollapseInfo new_collapse_info;
                                new_collapse_info.parent = this_parent;
                                new_collapse_info.count = 0;
                                collapseStack.push(new_collapse_info);
                                // Update cur_parent
                                cur_parent = this_parent;
                            }
                        }
                    }

                    // Now, if there's a match, increment. Finally, check if the
                    // top node can be cleaned out.
                    if (this_parent == cur_parent &&
                        this_parent != NULL)
                    {
                        collapseStack.top().count++;
                        if (collapseStack.top().count == collapseStack.top().parent->size()) {
                            // And this collapse is only useful if the parent is
                            // now *also* not satisfying the constraint.  This
                            // must also be true, otherwise we will constantly
                            // ping-pong back and forth between collapsing the
                            // node and expanding it back again.
                            bool parent_satisfies = this_parent->data().satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
                            visited++;
                            if (!parent_satisfies) {
                                // Note that the iterator returned is the parent
                                // cut node, so advancing the iterator at the
                                // end of this block is safe.
                                // We need an event for both with and without
                                // aggregates because the removal of a leaf node
                                // could affect the result set in either case.
                                QueryEventType evt;
                                it = replaceChildrenWithParent(it, &evt);
                                if (evt.size() > 0)
                                    events.push_back(evt);
                                // Since we've replaced the parent, we need to
                                // pop it off the candidate stack
                                // FIXME should check if it needs to push *it's*
                                // parent on the stack.
                                collapseStack.pop();
                            }
                        }
                    }

                    // And of course, we need to advance the cut node iterator
                    // and continue.
                    it++;
                    continue;
                }

                // What we do with satisfying nodes depends on whether they are
                // a leaf or not
                if (!node->leaf()) {
                    // For internal nodes that satisfy, we replace this node
                    // with the children. Note: don't increment since we need to
                    // start with the first child, which will now be
                    // it
                    if (parent->mWithAggregates) {
                        QueryEventType evt;
                        it = replaceParentWithChildren(it, &evt);
                        events.push_back(evt);
                    }
                    else {
                        it = replaceParentWithChildren(it, NULL);
                    }
                }
                else {
                    // For leaf nodes, there are two possibilities depending on
                    // whether we're tracking aggregates in the result set or
                    // not.
                    if (parent->mWithAggregates) {
                        // If we are tracking aggregates, we need to manage
                        // membership of the parent node.  last_satisfies and
                        // satisfies indicate whether a change occurred.

                        typename ResultSet::iterator result_it = results.find(node->rtnode->aggregateID());
                        bool in_results = (result_it != results.end());

                        // In either case, we're going to need to
                        // check if any of the children satisfy the
                        // constraint.
                        bool any_child_satisfied = false;
                        for(int i = 0; i < node->rtnode->size(); i++) {
                            ObjectID child_id = loc->iteratorID(node->rtnode->object(i).object);
                            bool child_satisfies = node->rtnode->childData(i,loc,t).satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
                            visited++;
                            if (child_satisfies) {
                                any_child_satisfied = true;
                                break;
                            }
                        }

                        if (!in_results && !any_child_satisfied) {
                            // If this node wasn't already in the
                            // results, then all the children should
                            // be. If no children satisfy anymore,
                            // lift the cut back up.
                            QueryEventType evt;
                            replaceLeafChildrenWithParent(node, &evt);
                            if (evt.size() > 0)
                                events.push_back(evt);

                        }
                        else if (in_results && any_child_satisfied) {
                            // If it is in the result set and some
                            // child did satisfy the constraint, we
                            // need to displace it with its children.
                            // With aggregates there's no need to
                            // check the objects for satisfying the
                            // query.
                            replaceParentWithChildrenResults(node);
                        }
                    }
                    else {
                        // And if we're not, then we always just add and remove
                        // individual objects from the result set based on
                        // whether they satsify the constraints.

                        // For leaves, we check each child and ensure its result set
                        // membership is correct.
                        for(int i = 0; i < node->rtnode->size(); i++) {
                            ObjectID child_id = loc->iteratorID(node->rtnode->object(i).object);
                            checkMembership(child_id, node->rtnode->childData(i, loc, t), qpos, qregion, qmaxsize, qangle, qradius);
                            visited++;
                        }
                    }

                    it++;
                }
            }

            validateCut();

            query->pushEvents(events);

            return visited;
        }

    }; // class Cut

    AggregateListenerType* aggregateListener() {
        return (mWithAggregates ? QueryHandlerType::mAggregateListener : NULL);
    }


    typedef typename RTreeCutQueryHandlerImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorImpl;
    friend class RTreeCutQueryHandlerImpl::NodeIteratorImpl<SimulationTraits>;

    virtual NodeIteratorImpl* nodesBeginImpl() const {
        return new NodeIteratorImpl(mRTree->nodesBegin());
    }
    virtual NodeIteratorImpl* nodesEndImpl() const {
        return new NodeIteratorImpl(mRTree->nodesEnd());
    }



    struct QueryState {
        QueryState(RTreeCutQueryHandler* _parent, QueryType* _query, RTreeNodeType* root)
        {
            cut = new Cut(_parent, _query, root);
        }

        ~QueryState() {
            delete cut;
        }

        Cut* cut;
    };

private:
    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, ObjectIDHasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;
    typedef typename QueryMap::const_iterator QueryMapConstIterator;

    LocationServiceCacheType* mLocCache;
    LocationUpdateProviderType* mLocUpdateProvider;
    ShouldTrackCallback mShouldTrackCB;

    RTree* mRTree;
    ObjectSet mObjects;
    QueryMap mQueries;
    Time mLastTime;
    uint16 mElementsPerNode;
    bool mWithAggregates;
    bool mRebuilding;
}; // class RTreeCutQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_CUT_QUERY_HANDLER_HPP_
