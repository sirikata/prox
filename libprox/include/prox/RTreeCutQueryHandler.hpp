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

#include <prox/QueryHandler.hpp>
#include <prox/LocationUpdateListener.hpp>
#include <prox/QueryChangeListener.hpp>
#include <prox/QueryCache.hpp>

#include <prox/DefaultSimulationTraits.hpp>

#include <prox/RTree.hpp>

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
            this,
            mElementsPerNode, mLocCache,
            static_objects,
            aggregateListener(),
            std::tr1::bind(&CutNode<SimulationTraits>::handleRootReplaced, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleSplit, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleLiftCut, _1, _2),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectInserted, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectRemoved, _1, _2)
        );
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

        // Start tracking all objects for second tree
        std::vector<LocCacheIterator> object_iterators;
        object_iterators.reserve( objects.size() );
        for(typename ObjectList::iterator it = objects.begin(); it != objects.end(); it++) {
            LocCacheIterator loc_it = mLocCache->startTracking(*it);
            object_iterators.push_back(loc_it);
            mObjects[*it] = loc_it;
        }

        // Build new tree
        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;
        using std::tr1::placeholders::_3;
        mRTree = new RTree(
            this,
            mElementsPerNode, mLocCache,
            static_objects,
            aggregateListener(),
            std::tr1::bind(&CutNode<SimulationTraits>::handleRootReplaced, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleSplit, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleLiftCut, _1, _2),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectInserted, _1, _2, _3),
            std::tr1::bind(&CutNode<SimulationTraits>::handleObjectRemoved, _1, _2)
        );
        mRTree->bulkLoad(object_iterators, mLastTime);

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

    virtual LocationServiceCacheType* locationCache() const {
        return mLocCache;
    }


    void addObject(const ObjectID& obj_id) {
        assert(mObjects.find(obj_id) == mObjects.end());

        mObjects[obj_id] = mLocCache->startTracking(obj_id);
        insertObj(obj_id, mLastTime);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
    }

    void removeObject(const ObjectID& obj_id) {
        typename ObjectSet::iterator it = mObjects.find(obj_id);
        if (it == mObjects.end()) return;

        mRTree->verifyConstraints(mLastTime);
        validateCuts();

        LocCacheIterator obj_loc_it = it->second;
        deleteObj(obj_id, mLastTime);
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
            retval.push_back(it->first);
        return retval;
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

    void deleteObj(const ObjectID& obj_id, const Time& t) {
        assert(mObjects.find(obj_id) != mObjects.end());
        mRTree->erase(mObjects[obj_id], t);
    }

	///this needs to be a template class for no good reason: Microsoft visual studio bugs demand it.
    template <class XSimulationTraits>struct CutNode;
    struct Cut;

#if RTREE_DATA == RTREE_DATA_BOUNDS
    typedef BoundingSphereData<SimulationTraits, CutNode<SimulationTraits> > NodeData;
#elif RTREE_DATA == RTREE_DATA_MAXSIZE
    typedef MaxSphereData<SimulationTraits, CutNode<SimulationTraits> > NodeData;
#endif
    typedef Prox::RTree<SimulationTraits, NodeData, CutNode<SimulationTraits> > RTree;
    typedef typename RTree::RTreeNodeType RTreeNodeType;

    ///this needs to be a template class for no good reason: Microsoft visual studio bugs demand it.
    template <class XSimulationTraits>struct CutNode {
        typedef Cut CutType;

        Cut* parent;
		typedef typename Prox::RTree<XSimulationTraits, NodeData, CutNode<SimulationTraits> >::RTreeNodeType RTreeNodeType;
		RTreeNodeType * rtnode;
        bool satisfies;

        CutNode(QueryHandlerType* handler, Cut* _parent, RTreeNodeType* _rt, AggregateListenerType* listener)
         : parent(_parent),
           rtnode(_rt),
           satisfies(false)
        {
            rtnode->insertCutNode(this);
            if (listener != NULL) listener->aggregateObserved(handler, rtnode->aggregateID(), rtnode->cutNodesSize());
        }

        void destroy(QueryHandlerType* handler, AggregateListenerType* listener) {
            rtnode->eraseCutNode(this);
            if (listener != NULL) listener->aggregateObserved(handler, rtnode->aggregateID(), rtnode->cutNodesSize());
            delete this;
        }
    private:
        ~CutNode() {
        }

    public:

        bool updateSatisfies(const Vector3& qpos, const BoundingSphere& qregion, float qmaxsize, const SolidAngle& qangle, float qradius) {
            satisfies = rtnode->data().satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
            return satisfies;
        }

        bool leaf() const {
            return rtnode->leaf();
        }


        void handleRootReplaced(RTreeNodeType* orig_root, RTreeNodeType* new_root) {
            parent->handleRootReplaced(this, orig_root, new_root);
        }
        void handleSplit(RTreeNodeType* orig_node, RTreeNodeType* new_node) {
            parent->handleSplit(this, orig_node, new_node);
        }
        void handleLiftCut(RTreeNodeType* to_node) {
            parent->handleLiftCut(this, to_node);
        }
        void handleObjectInserted(const LocCacheIterator& objit, int objidx) {
            parent->handleObjectInserted(this, objit, objidx);
        }
        void handleObjectRemoved(const LocCacheIterator& objit) {
            parent->handleObjectRemoved(this, objit);
        }
    };

    struct Cut {
    private:
        Cut();

        RTreeCutQueryHandler* parent;
        QueryType* query;
        // A cut is made up of a list of CutNodes
        typedef std::list<CutNode<SimulationTraits>*> CutNodeList;
        typedef typename CutNodeList::iterator CutNodeListIterator;
        typedef typename CutNodeList::const_iterator CutNodeListConstIterator;
        CutNodeList nodes;
        int32 length;

        typedef std::tr1::unordered_set<ObjectID, ObjectIDHasher> ResultSet;
        ResultSet results;

        typedef std::deque<QueryEventType> EventQueue;
        EventQueue events;

        QueryEventType swapEvent;

        // Checks for child_id's membership in the result set.  This version
        // should be used for non-aggregate queries.
        void checkMembership(const ObjectID& child_id, const NodeData& child_data, const Vector3& qpos, const BoundingSphere& qregion, float qmaxsize, const SolidAngle& qangle, float qradius) {
            bool child_satisfies = child_data.satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
            typename ResultSet::iterator result_it = results.find(child_id);
            bool in_results = (result_it != results.end());
            if (child_satisfies && !in_results) {
                results.insert(child_id);

                QueryEventType evt;
                evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal) );
                events.push_back(evt);
            }
            else if (!child_satisfies && in_results) {
                results.erase(result_it);

                QueryEventType evt;
                evt.removals().push_back( typename QueryEventType::Removal(child_id, QueryEventType::Normal) );
                events.push_back(evt);
            }
        }

        CutNodeListIterator replaceParentWithChildren(const CutNodeListIterator& parent_it, QueryEventType* qevt_out) {
            CutNode<SimulationTraits>* parent_cn = *parent_it;
            assert(!parent_cn->leaf());
            // Inserts before, so get next it
            CutNodeListIterator next_it = parent_it;
            next_it++;
            // Insert all new nodes. Going backwards leaves next_it as first of
            // new elements
            for(int i = parent_cn->rtnode->size()-1; i >=0; i--) {
                RTreeNodeType* child_rtnode = parent_cn->rtnode->node(i);
                if (qevt_out) {
                    qevt_out->additions().push_back( typename QueryEventType::Addition(child_rtnode->aggregateID(), QueryEventType::Imposter) );
                    results.insert(child_rtnode->aggregateID());
                }
                next_it = nodes.insert(next_it, new CutNode<SimulationTraits>(parent, this, child_rtnode, parent->aggregateListener()));
            }
            // Delete old node
            if (qevt_out) {
                qevt_out->removals().push_back( typename QueryEventType::Removal(parent_cn->rtnode->aggregateID(), QueryEventType::Imposter) );
                results.erase(parent_cn->rtnode->aggregateID());
            }
            nodes.erase(parent_it);
            length += (parent_cn->rtnode->size()-1);
            // And clean up
            parent_cn->destroy(parent, parent->aggregateListener());

            return next_it;
        }

        // Replace the children of a leaf node (i.e. objects) with the
        // node itself.  Just adjusts the result set since
        void replaceLeafChildrenWithParent(CutNode<SimulationTraits>* cnode, QueryEventType* qevt_out) {
            RTreeNodeType* node = cnode->rtnode;
            assert(node->leaf());
            // At leaves, if the aggregate wasn't in the results (either
            // because it had been refined or because we're not returning
            // aggregates), we need to check for children in the result set.

            // FIXME for sanity checking we could track # of removed
            // children when mWithAggregates is true and validate that
            // it is the same as the total number of children
            for(int leafidx = 0; leafidx < node->size(); leafidx++) {
                ObjectID leaf_id = parent->mLocCache->iteratorID(node->object(leafidx).object);
                size_t n_leaf_removed = results.erase(leaf_id);
                if (n_leaf_removed > 0)
                    qevt_out->removals().push_back( typename QueryEventType::Removal(leaf_id, QueryEventType::Normal) );
            }

            qevt_out->additions().push_back( typename QueryEventType::Addition(node->aggregateID(), QueryEventType::Imposter) );
            results.insert(node->aggregateID());
            // Note: no modification of length because we haven't
            // actually added or removed anything, only adjusted the
            // result set.
        }

        // Replaces children with parent in a cut.  Returns an iterator to the
        // new parent node.
        CutNodeListIterator replaceChildrenWithParent(const CutNodeListIterator& last_child_it, QueryEventType* qevt_out) {
            CutNodeListIterator child_it = last_child_it;
            RTreeNodeType* parent_rtnode = (*child_it)->rtnode->parent();
            int nchildren = parent_rtnode->size();

            // Add the new node using the parent.
            if (parent->mWithAggregates) {
                qevt_out->additions().push_back( typename QueryEventType::Addition(parent_rtnode->aggregateID(), QueryEventType::Imposter) );
                results.insert(parent_rtnode->aggregateID());
            }
            // Parent needs to be inserted after children, insert puts it before
            // the iterator passed in.
            CutNodeListIterator parent_insert_it = child_it;
            parent_insert_it++;
            CutNodeListIterator parent_it = nodes.insert(parent_insert_it, new CutNode<SimulationTraits>(parent, this, parent_rtnode, parent->aggregateListener()));

            // Work backwards removing all the children.
            for(int i = nchildren-1; i >=0; i--) {
                CutNode<SimulationTraits>* child_cn = (*child_it);
                RTreeNodeType* child_rtnode = child_cn->rtnode;
                assert(child_rtnode->parent() == parent_rtnode);
                assert(parent_rtnode->node(i) == child_rtnode);

                bool aggregate_was_in_results = false;
                // Only try to remove the child node from results for aggregates
                if (parent->mWithAggregates) {
                    size_t nremoved = results.erase(child_rtnode->aggregateID());
                    if (nremoved > 0) {
                        aggregate_was_in_results = true;
                        qevt_out->removals().push_back( typename QueryEventType::Removal(child_rtnode->aggregateID(), QueryEventType::Imposter) );
                    }
                }
                // At leaves, if the aggregate wasn't in the results (either
                // because it had been refined or because we're not returning
                // aggregates), we need to check for children in the
                // result set.
                // This is almost like replaceLeafChildrenWithParent
                // but doesn't add the parent since we're in the
                // process of removing it.
                if (!aggregate_was_in_results && child_rtnode->leaf()) {
                    // FIXME for sanity checking we could track # of removed
                    // children when mWithAggregates is true and validate that
                    // it is the same as the total number of children
                    for(int leafidx = 0; leafidx < child_rtnode->size(); leafidx++) {
                        ObjectID leaf_id = parent->mLocCache->iteratorID(child_rtnode->object(leafidx).object);
                        size_t n_leaf_removed = results.erase(leaf_id);
                        if (n_leaf_removed > 0)
                            qevt_out->removals().push_back( typename QueryEventType::Removal(leaf_id, QueryEventType::Normal) );
                    }
                }

                // Erase and clean up the child. Returns *next* element, so move
                // backwards to get previous child.
                child_it = nodes.erase(child_it);
                child_cn->destroy(parent, parent->aggregateListener());
                // We should only be able to hit nodes.begin() if we've removed
                // the last child *and* these children were the start of the cut
                assert(child_it != nodes.begin() || i == 0);
                // i > 0 is just a faster check for most iterations
                if (i > 0 || child_it != nodes.begin())
                    child_it--;
            }

            length -= (parent_rtnode->size()-1);

            return parent_it;
        }

        // Replaces a parent node with children objects in the result set.  This
        // only makes sense for aggregates.  It should be used when one of the
        // child objects satisfies the constraints and therefore pulls all the
        // children in with it. Should not be used for non-leaf nodes.
        void replaceParentWithChildrenResults(CutNode<SimulationTraits>* cnode) {
            QueryEventType evt;
            for(int i = 0; i < cnode->rtnode->size(); i++) {
                ObjectID child_id = parent->mLocCache->iteratorID(cnode->rtnode->object(i).object);
                results.insert(child_id);
                evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal) );
            }
            // For some reason this:
            //results.erase(result_it);
            // is breaking, even though I can't see how
            //result_it could ever be invalid. Instead, do
            //it the hard way and assert:
            size_t nremoved = results.erase(cnode->rtnode->aggregateID());
            assert(nremoved == 1);
            evt.removals().push_back( typename QueryEventType::Removal(cnode->rtnode->aggregateID(), QueryEventType::Imposter) );
            events.push_back(evt);
        }

        static RTreeNodeType* _get_root(RTreeNodeType* node) {
            RTreeNodeType* n = node;
            while(n->parent() != NULL) {
                n = n->parent();
            }
            return n;
        }

        static bool _is_ancestor(RTreeNodeType* node, RTreeNodeType* anc) {
            if (node == anc) return true;

            RTreeNodeType* n = node;
            while(n->parent() != NULL) {
                RTreeNodeType* parent = n->parent();
                if (parent == anc) return true;
                n = parent;
            }
            return false;
        }

        void removeObjectChildFromResults(const ObjectID& child_id) {
            typename ResultSet::iterator result_it = results.find(child_id);
            bool in_results = (result_it != results.end());
            if (in_results) {
                results.erase(result_it);

                QueryEventType evt;
                evt.removals().push_back( typename QueryEventType::Removal(child_id, QueryEventType::Normal) );
                events.push_back(evt);
            }
        }

        void removeObjectChildrenFromResults(RTreeNodeType* from_node) {
            // Notify any cuts that objects held by this node are gone
            assert(from_node->leaf());
            for(typename RTreeNodeType::Index idx = 0; idx < from_node->size(); idx++) {
                removeObjectChildFromResults( parent->mLocCache->iteratorID(from_node->object(idx).object) );
            }
        }

        // Utility that removes and destroys a cut node, and removes results it
        // had triggered from the result set.
        void destroyCutNode(CutNode<SimulationTraits>* node, QueryEventType& evt) {
            if (parent->mWithAggregates) {
                // When dealing with aggregates, we first check if the
                // node itself is in the result set since if it is, none
                // of its children can be (if it is a leaf).
                // set.
                size_t nremoved = results.erase(node->rtnode->aggregateID());
                if (nremoved > 0) {
                    evt.removals().push_back( typename QueryEventType::Removal(node->rtnode->aggregateID(), QueryEventType::Imposter) );
                }
                else {
                    // If it wasn't there and this is a leaf, we need to
                    // check for children in the result set.  In this
                    // case, they should all be there.
                    removeObjectChildrenFromResults(node->rtnode);
                }
            }
            else {
                // Without aggregates, we only need to check to remove
                // children from the result set if we're at a leaf.  In
                // this case, some may be there, some may not.
                if (node->rtnode->leaf())
                    removeObjectChildrenFromResults(node->rtnode);
            }
            node->destroy(parent, parent->aggregateListener());
        }

        void validateCutNodesInRTreeNodes() const {
            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode<SimulationTraits>* node = *it;
                RTreeNodeType* rtnode = node->rtnode;
                assert(rtnode->findCutNode(node) != rtnode->cutNodesEnd());
            }
        }

        void validateCutNodesInTree() const {
            // Get the root base on the first cut node.  Even if this one is
            // broken, we'll be able to tell that the trees have become disjoint
            CutNode<SimulationTraits>* first_cut_node = *(nodes.begin());
            RTreeNodeType* root = _get_root(first_cut_node->rtnode);

            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode<SimulationTraits>* node = *it;
                assert(_is_ancestor(node->rtnode, root));
            }
        };

        // Validates that cut nodes are not through RTree nodes that are
        // ancestors of each other.
        void validateCutNodesUnrelated() const {
            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode<SimulationTraits>* node = *it;
                for(CutNodeListConstIterator other_it = nodes.begin(); other_it != nodes.end(); other_it++) {
                    CutNode<SimulationTraits>* other_node = *other_it;
                    if (node == other_node) continue;
                    assert( ! _is_ancestor(node->rtnode, other_node->rtnode) );
                    assert( ! _is_ancestor(other_node->rtnode, node->rtnode) );
                }
            }
        };

        // Rebuild an ordered cut. Works recursively.
        //
        // Note that in PROXDEBUG mode this also verifies, if a cut node was
        // found in an rtree node, that processing children doesn't change the
        // size of the cut (i.e. we didn't find a cutnode in a subtree where we
        // shouldn't).  This is more expensive, but also covers the
        // functionality of validateCutNodesUnrelated.
        void rebuildOrderedCut(CutNodeList& inorder, RTreeNodeType* root) {
            bool had_cut = false;
            typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(this);
            if (node_its != root->cutNodesEnd()) {
                CutNode<SimulationTraits>* cnode = node_its->second;
                assert(cnode->parent == this);
                inorder.push_back(cnode);
                had_cut = true;
            }

            if (root->leaf())
                return;

            // With PROXDEBUG we verify no children get added if we process
            // children nodes, i.e. that we don't have sibling cutnodes in
            // ancestor rtree nodes.  Without PROXDEBUG, only processes children
            // nodes if a cut node wasn't found at this node, efficiently
            // culling the tree.
#if !defined(PROXDEBUG)
            if (!had_cut) {
#else
                int num_before_children = inorder.size();
#endif

                for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                    rebuildOrderedCut(inorder, root->node(i));
#if defined(PROXDEBUG)
                int num_after_children = inorder.size();
                assert(!had_cut || num_before_children == num_after_children);
#else
            }
#endif
        };


        // Helper for rebuildOrderedCutWithViolations. Looks for CutNodes in a
        // subtree and a) removes them and b) removes any results they had from
        // the result set.
        void rebuildOrderedCutWithViolations_removeChildrenCutNodes(QueryEventType& evt, RTreeNodeType* root) {
            // First, check for a cut node at this bvh node
            typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(this);
            if (node_its != root->cutNodesEnd()) {
                CutNode<SimulationTraits>* cnode = node_its->second;
                assert(cnode->parent == this);
                destroyCutNode(cnode, evt);
            }

            // Then, recurse and check within children
            if (root->leaf()) return;
            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                rebuildOrderedCutWithViolations_removeChildrenCutNodes(evt, root->node(i));
        };

        // Driver for rebuildOrderedCutWithViolations first pass. Scans the tree
        // with a pre-order traversal to filter out cut nodes that appear
        // beneath other cut nodes.
        void rebuildOrderedCutWithViolations_filterChildrenPass(QueryEventType& evt, RTreeNodeType* root) {
            typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(this);

            if (root->leaf()) return;

            // If there's a cut node here, remove all children cut nodes
            if (node_its != root->cutNodesEnd()) {
                for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                    rebuildOrderedCutWithViolations_removeChildrenCutNodes(evt, root->node(i));
            } // Otherwise, recurse
            else {
                for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                    rebuildOrderedCutWithViolations_filterChildrenPass(evt, root->node(i));
            }
        };

        // Driver for rebuildOrderedCutWithViolations second pass. Scans through
        // looking for gaps and inserts new CutNodes.
        bool rebuildOrderedCutWithViolations_fillGapsPass(QueryEventType& evt, RTreeNodeType* root, bool treat_as_root = false) {
            // The basic approach is to process all children of this node
            // recursively and record whether the child had data filled in
            // (returned true).  If no children found CutNodes, then no data
            // would be filled in and we just return, leaving the cut to be
            // handled at a higher level.  If any children did have nodes, then
            // those that returned that they were empty get a cut node and those
            // that returned that they found a cut node should already be
            // filled.

            // If there's a cut node of ours on this node, we're done with this subtree
            typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(this);
            if (node_its != root->cutNodesEnd()) return true;

            // Base case: at a leaf, there's no additional processing to be
            // done. This subtree is empty.
            if (root->leaf()) return false;

            // Next, process each of the children, recording whether they are
            // filled or not.
            std::vector<bool> children_results;
            bool any_child_was_filled = false;
            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++) {
                bool child_was_filled = rebuildOrderedCutWithViolations_fillGapsPass(evt, root->node(i));
                children_results.push_back(child_was_filled);
                any_child_was_filled = (any_child_was_filled || child_was_filled);
            }

            // Now, either everything was empty...
            if (!any_child_was_filled) {
                // If we're the root and all children were empty, fill in a
                // CutNode for us.
                if (treat_as_root) {
                    CutNode<SimulationTraits>* new_cnode = new CutNode<SimulationTraits>(parent, this, root, parent->aggregateListener());
                    if (parent->mWithAggregates) {
                        evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode->aggregateID(), QueryEventType::Imposter) );
                        results.insert(new_cnode->rtnode->aggregateID());
                    }
                }
                return false;
            }
            // Or we need to fill in the empties
            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++) {
                if (children_results[i] == true) // Already filled
                    continue;
                // Add a CutNode for this child
                CutNode<SimulationTraits>* new_cnode = new CutNode<SimulationTraits>(parent, this, root->node(i), parent->aggregateListener());
                if (parent->mWithAggregates) {
                    evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode->aggregateID(), QueryEventType::Imposter) );
                    results.insert(new_cnode->rtnode->aggregateID());
                }
                // Not adding to list since we're rebuilding it in the next pass
            }
            return true;
        }

        // Rebuild an ordered cut. Works recursively. Handles cut nodes that
        // have been resorted such that there are parent/child violations and
        // full cut violations: cut nodes may have been reordered such that
        // there are gaps and there are cut nodes in subtrees of other cut
        // nodes.
        void rebuildOrderedCutWithViolations(CutNodeList& inorder, RTreeNodeType* root) {
            // This works in two passes.
            QueryEventType evt;
            // On the first pass, we make sure we don't have any overlapping cut
            // nodes, i.e. that reordering hasn't caused the rtnode of one cut
            // node to become the child of the rtnode of another cut node.
            rebuildOrderedCutWithViolations_filterChildrenPass(evt, root);
            // On the second pass, we look for gaps and fill them in with new
            // CutNodes. Last parameter indicates that, if all children were
            // empty we should treat this as the root and make sure a cut node
            // exists.
            rebuildOrderedCutWithViolations_fillGapsPass(evt, root, true);
            // On the third pass, we actually rebuild the cut.  This just uses
            // the normal approach since the previous passes guarantee gap-free,
            // non-overlapping CutNodes.
            rebuildOrderedCut(inorder, root);
            // Save the adjustments triggered by this.
            if (evt.size() > 0)
                events.push_back(evt);
        }


        // Validates that the nodes in a cut are in order as they cut across the
        // nodes of the RTree. This is a necessary condition for the cuts to get
        // pushed up properly.
        void validateCutOrdered() {
            // There's almost certainly a more efficient way to do this, but the
            // easiest way is to build a new list by exploring the tree in-order
            // for nodes
            CutNodeList nodes_inorder;
            rebuildOrderedCut(nodes_inorder, parent->mRTree->root());
            assert(nodes_inorder.size() == nodes.size());
            for(CutNodeListConstIterator it = nodes.begin(), other_it = nodes_inorder.begin();
                it != nodes.end(); it++, other_it++) {
                CutNode<SimulationTraits>* node = *it;
                CutNode<SimulationTraits>* othernode = *other_it;
                assert(node == othernode);
            }
        };

        // Validates that all entries in a subtree (aggregates and
        // non-aggregates) are *not* in the result set.
        void validateSubtreeObjectsNotInResults(RTreeNodeType* root) {
            assert(results.find(root->aggregateID()) == results.end());
            validateChildrenSubtreesObjectsNotInResults(root);
        }

        void validateChildrenSubtreesObjectsNotInResults(RTreeNodeType* root) {
            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++) {
                if (root->leaf())
                    assert( results.find(parent->mLocCache->iteratorID(root->object(i).object)) == results.end());
                else
                    validateSubtreeObjectsNotInResults(root->node(i));
            }
        }

        // Validates that the result set matches the nodes the cut goes through,
        // checking both for missing entries (e.g. one leaf object out of 5
        // children is missing) and for extra entries (e.g. a cut moved, but
        // somebody neglected to remove the entry or child objects' entries from
        // results).
        void validateResultsMatchCut() {
            ResultSet accounted;
            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode<SimulationTraits>* node = *it;
                RTreeNodeType* rtnode = node->rtnode;
                if (parent->mWithAggregates) {
                    // Check for the aggregate and invalidate children
                    if (results.find(rtnode->aggregateID()) != results.end()) {
                        accounted.insert(rtnode->aggregateID());
                        validateChildrenSubtreesObjectsNotInResults(rtnode);
                    }
                    else { // Otherwise, we better have all the children
                        for(typename RTreeNodeType::Index i = 0; i < rtnode->size(); i++)
                            accounted.insert( rtnode->leaf() ? parent->mLocCache->iteratorID(rtnode->object(i).object) : rtnode->node(i)->aggregateID() );
                    }
                }
                else {
                    // Without aggregates, we should have some subset of the
                    // children of the node.
                    if (!rtnode->leaf()) continue;
                    // To avoid actually evaluating, we're conservative in this
                    // case and might miss some false positives. We just add all
                    // leaf children we encounter
                    for(typename RTreeNodeType::Index i = 0; i < rtnode->size(); i++)
                        accounted.insert( parent->mLocCache->iteratorID(rtnode->object(i).object) );
                }
            }

            // Now that we've collected the information, we can report errors.

            // Accounted - results = objects that are missing from the results
            // We can only do this with aggregates since we had to be
            // conservative with non-aggregates.
            if (parent->mWithAggregates) {
                for(typename ResultSet::iterator it = accounted.begin(); it != accounted.end(); it++)
                    assert( results.find(*it) != results.end() );
            }
            // Results - accounted = extra objects in the result set
            for(typename ResultSet::iterator it = results.begin(); it != results.end(); it++)
                assert( accounted.find(*it) != accounted.end() );
        }

    public:

        /** Regular constructor.  A new cut simply starts with the root node and
         *  immediately refines.
         */
        Cut(RTreeCutQueryHandler* _parent, QueryType* _query, RTreeNodeType* root)
         : parent(_parent),
           query(_query)
        {
            if (parent->mWithAggregates) {
                QueryEventType evt;
                evt.additions().push_back( typename QueryEventType::Addition(root->aggregateID(), QueryEventType::Imposter) );
                results.insert(root->aggregateID());
                events.push_back(evt);
            }
            nodes.push_back(new CutNode<SimulationTraits>(parent, this, root, parent->aggregateListener()));

            length = 1;
            validateCut();
        }

        ~Cut() {
            for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode<SimulationTraits>* node = *it;
                node->destroy(parent, parent->aggregateListener());
            }
            nodes.clear();
            length = 0;
        }

        void validateCut() {
#ifdef PROXDEBUG
            assert(length == nodes.size());
            validateCutNodesInRTreeNodes();
            validateCutNodesInTree();
            // Now covered by validateCutOrdere
            //validateCutNodesUnrelated();
            validateCutOrdered();
            validateResultsMatchCut();
#endif //PROXDEBUG
        };

        int cutSize() const {
            assert(length == (int)nodes.size());
            return length;
        };

        int resultsSize() const {
            return results.size();
        }
    private:
        // Struct which keeps track of how many children do not satisfy the
        // constraint so they can be collapsed.
        struct ParentCollapseInfo {
            RTreeNodeType* parent;
            int count;
        };

    public:

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
                            evt.additions().push_back( typename QueryEventType::Addition(node->rtnode->aggregateID(), QueryEventType::Imposter) );
                            results.insert( node->rtnode->aggregateID() );
                            for(int i = 0; i < node->rtnode->size(); i++) {
                                ObjectID child_id = loc->iteratorID(node->rtnode->object(i).object);
                                typename ResultSet::iterator result_it = results.find(child_id);
                                assert(result_it != results.end());
                                results.erase(result_it);
                                evt.removals().push_back( typename QueryEventType::Removal(child_id, QueryEventType::Normal) );
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

        void handleRootReplaced(CutNode<SimulationTraits>* cnode, RTreeNodeType* orig_root, RTreeNodeType* new_root) {
            // The old root was replaced by the new root because the tree is
            // getting smaller.  We just need to shift our cut down to the new
            // node.
            // FIXME linear search could be avoided by storing iterators
            CutNodeListIterator it = std::find(nodes.begin(), nodes.end(), cnode);
            if (parent->mWithAggregates) {
                QueryEventType evt;
                it = replaceParentWithChildren(it, &evt);
                events.push_back(evt);
            }
            else {
                it = replaceParentWithChildren(it, NULL);
            }
        }

        // Handle a split of orig_node into orig_node and new_node. cnode is the
        // CutNode that was (and remains) at orig_node.
        void handleSplit(CutNode<SimulationTraits>* cnode, RTreeNodeType* orig_node, RTreeNodeType* new_node) {
            // Add a new CutNode to new_node and insert it in our cut list.
            // Future updates will take care of any additional changes (push up
            // or down) that still need to be applied to the tree.

            // FIXME we could avoid this linear search by storing iterators in CutNode
            CutNodeListIterator orig_list_it = std::find(nodes.begin(), nodes.end(), cnode);
            assert(orig_list_it != nodes.end());
            CutNodeListIterator after_orig_list_it = orig_list_it; after_orig_list_it++;

            CutNode<SimulationTraits>* new_cnode = new CutNode<SimulationTraits>(parent, this, new_node, parent->aggregateListener());
            if (parent->mWithAggregates) {
                QueryEventType evt;
                evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode->aggregateID(), QueryEventType::Imposter) );
                results.insert(new_cnode->rtnode->aggregateID());
                events.push_back(evt);
            }
            nodes.insert(after_orig_list_it, new_cnode);
            length++;

            // Mid-operation, no validation
        }

        void handleLiftCut(CutNode<SimulationTraits>* cnode, RTreeNodeType* to_node) {
            validateCut();

            // This is tricky. The cutnode may be nowhere near the node we need
            // to pull up to. Instead, we have to find all cut nodes whose nodes
            // are children of to_node, destroy them, and replace them with a
            // single cut node at to_node.

            QueryEventType evt;

            // We'll exit when we have last_was_ancestor == true and
            // _is_ancestor == false, indicating we hit the end of the run for
            // this parent node.
            bool last_was_ancestor = false;
            CutNodeListIterator it;
            for(it = nodes.begin(); it != nodes.end(); ) {
                CutNode<SimulationTraits>* node = *it;

                if ( _is_ancestor(node->rtnode, to_node) ) {
                    last_was_ancestor = true;
                    it = nodes.erase(it);
                    length--;
                    destroyCutNode(node, evt);
                }
                else {
                    // If the last one was a child and we aren't then we can
                    // stop traversing.  We don't advance the iterator because
                    // leaving it here allows us to insert before the first
                    // non-child, which should be the right place.
                    if (last_was_ancestor)
                        break;
                    it++;
                }
            }

            // New node insertion must happen at the end to avoid removing the
            // new node
            // NOTE: We always do this because the callback shouldn't even be
            // called unless we needed to remove one of these.
            CutNode<SimulationTraits>* new_cnode = new CutNode<SimulationTraits>(parent, this, to_node, parent->aggregateListener());
            if (parent->mWithAggregates) {
                evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode->aggregateID(), QueryEventType::Imposter) );
                results.insert(new_cnode->rtnode->aggregateID());
            }
            nodes.insert(it, new_cnode);
            length++;

            if (parent->mWithAggregates)
                events.push_back(evt);

            validateCut();
        }

        void handleObjectInserted(CutNode<SimulationTraits>* cnode, const LocCacheIterator& objit, int objidx) {
            // Ignore insertions/deletions during rebuild
            if (parent->mRebuilding) return;

            RTreeNodeType* node = cnode->rtnode;
            assert(node->leaf());

            if (parent->mWithAggregates) {
                // When dealing with aggregates, since this node is on the cut
                // and a leaf, there are two possibilities -- the node is truly
                // in the results and none of its children are or the node is
                // not in the results and all of its children are.
                //
                // Therefore, we only have 2 choices.  If the node is in the
                // result set, we need to check the child and possibly push the
                // cut down.  If the node is not, we must simply add the child
                // as a new result.
                typename ResultSet::iterator parent_result_it = results.find(node->aggregateID());
                bool parent_in_results = (parent_result_it != results.end());

                if (!parent_in_results) {
                    // Just add the child
                    ObjectID child_id = parent->mLocCache->iteratorID(objit);
                    assert(results.find(child_id) == results.end());

                    results.insert(child_id);

                    QueryEventType evt;
                    evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal) );
                    events.push_back(evt);
                }
                else {
                    // Check this child to decide whether to replace parent with
                    // children
                    Time t = parent->mLastTime;
                    Vector3 qpos = query->position(t);
                    BoundingSphere qregion = query->region();
                    float qmaxsize = query->maxSize();
                    const SolidAngle& qangle = query->angle();
                    float qradius = query->radius();

                    ObjectID child_id = parent->mLocCache->iteratorID(objit);
                    bool child_satisfies = node->childData(objidx, parent->mLocCache, t).satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
                    if (child_satisfies) {
                        replaceParentWithChildrenResults(cnode);
                    }
                }
            }
            else {
                // If we're not dealing with aggregates, we just need to check
                // if we should be adding this to the result set immediately.
                Time t = parent->mLastTime;
                Vector3 qpos = query->position(t);
                BoundingSphere qregion = query->region();
                float qmaxsize = query->maxSize();
                const SolidAngle& qangle = query->angle();
                float qradius = query->radius();

                ObjectID child_id = parent->mLocCache->iteratorID(objit);
                checkMembership(child_id, node->childData(objidx, parent->mLocCache, t), qpos, qregion, qmaxsize, qangle, qradius);
            }
        }

        void handleObjectRemoved(CutNode<SimulationTraits>* cnode, const LocCacheIterator& objit) {
            // Ignore insertions/deletions during rebuild
            if (parent->mRebuilding) return;

            // We just need to remove the object from the result set if we have
            // it.
            ObjectID child_id = parent->mLocCache->iteratorID(objit);
            removeObjectChildFromResults(child_id);

            validateCut();
        }

        /** Rebuilds and actually replaces the cut node list.  Assumes that no
         *  CutNodes have become invalidated, only that they've become jumbled.
         *  This can be used when the tree is reorganized such that nodes shift
         *  position, but the overall topology of the tree does not change,
         *  e.g. if nodes are reordered for better coherence, but all the nodes
         *  should still be valid.
         */
        void rebuildCutOrder() {
            CutNodeList in_order;
            rebuildOrderedCutWithViolations(in_order, parent->mRTree->root());
            nodes.swap(in_order);
            length = nodes.size();
            //validateCut();
        }


        // Fills in an event that corresponds to destroying the entire cut.
        void destroyCut(QueryEventType& destroyEvent) {
            // Run through the cut, adding removals to the result event
            for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode<SimulationTraits>* cnode = *it;
                RTreeNodeType* node = cnode->rtnode;

                // Try to remove the node itself
                size_t node_removed = results.erase(node->aggregateID());
                if (node_removed > 0)
                    destroyEvent.removals().push_back( typename QueryEventType::Removal(node->aggregateID(), QueryEventType::Imposter) );

                // And, if its a leaf, try to remove its children
                if (node->leaf() && node_removed == 0) {
                    for(int leaf_idx = 0; leaf_idx < node->size(); leaf_idx++) {
                        ObjectID leaf_id = parent->mLocCache->iteratorID(node->object(leaf_idx).object);
                        size_t leaf_removed = results.erase(leaf_id);
                        if (leaf_removed > 0)
                            destroyEvent.removals().push_back( typename QueryEventType::Removal(leaf_id, QueryEventType::Normal) );
                    }
                }

                // And remove the cut node
                cnode->destroy(parent, parent->aggregateListener());
            }
            nodes.clear();
            length = 0;
        }

        // In order to swap trees, we need to get cuts out of the way and
        // replace them with the root of the new tree.  In order to not have
        // both trees in memory at the same time, we split this into two
        // phases. The first removes the cut from the original tree, the second
        // finishes the process by adding it to the new tree and adding the
        // updates to the result set.

        void startSwapTrees() {
            destroyCut(swapEvent);
        }

        void finishSwapTrees(RTreeNodeType* new_root) {
            // Add in the root CutNode
            if (parent->mWithAggregates) {
                swapEvent.additions().push_back( typename QueryEventType::Addition(new_root->aggregateID(), QueryEventType::Imposter) );
                results.insert(new_root->aggregateID());
            }
            nodes.push_back(new CutNode<SimulationTraits>(parent, this, new_root, parent->aggregateListener()));
            length = 1;
            validateCut();

            // And finally, we can push the event onto the queue.
            if (!swapEvent.empty()) {
                events.push_back(swapEvent);
                swapEvent = QueryEventType();
            }
        }

    };


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

    AggregateListenerType* aggregateListener() {
        return (mWithAggregates ? QueryHandlerType::mAggregateListener : NULL);
    }

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
