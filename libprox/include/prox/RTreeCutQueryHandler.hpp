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
    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

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

    RTreeCutQueryHandler(uint16 elements_per_node)
     : QueryHandlerType(),
       mLocCache(NULL),
       mRTree(NULL),
       mLastTime(Time::null()),
       mElementsPerNode(elements_per_node)
    {
    }

    virtual ~RTreeCutQueryHandler() {
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

    void initialize(LocationServiceCacheType* loc_cache) {
        mLocCache = loc_cache;
        mLocCache->addUpdateListener(this);

        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;
        using std::tr1::placeholders::_3;

        mRTree = new RTree(
            mElementsPerNode, mLocCache,
            std::tr1::bind(&CutNode::handleSplit, _1, _2, _3),
            std::tr1::bind(&CutNode::handleLiftCut, _1, _2),
            std::tr1::bind(&CutNode::handleObjectRemoved, _1, _2)
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

    void tick(const Time& t) {
        mRTree->update(t);

        mRTree->verifyConstraints(t);
        validateCuts();
        int count = 0;
        int ncount = 0;
        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            QueryType* query = query_it->first;
            QueryState* state = query_it->second;

            state->cut->update(mLocCache, t);
        }
        mLastTime = t;

        mRTree->verifyConstraints(t);
        validateCuts();
    }

    virtual uint32 numObjects() const {
        return (uint32)mObjects.size();
    }
    virtual uint32 numQueries() const {
        return (uint32)mQueries.size();
    }

    void locationConnected(const ObjectID& obj_id, const MotionVector3& pos, const BoundingSphere& region, Real ms) {
        assert(mObjects.find(obj_id) == mObjects.end());
        mObjects[obj_id] = mLocCache->startTracking(obj_id);
        insertObj(obj_id, mLastTime);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
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
        mRTree->verifyConstraints(mLastTime);
        validateCuts();

        assert( mObjects.find(obj_id) != mObjects.end() );
        LocCacheIterator obj_loc_it = mObjects[obj_id];
        deleteObj(obj_id, mLastTime);
        mLocCache->stopTracking(obj_loc_it);
        mObjects.erase(obj_id);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
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

    void queryDeleted(const QueryType* query) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;
        delete state;
        mQueries.erase(it);

        mRTree->verifyConstraints(mLastTime);
        validateCuts();
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
    void insertObj(const ObjectID& obj_id, const Time& t) {
        mRTree->insert(mObjects[obj_id], t);
    }

    void updateObj(const ObjectID& obj_id, const Time& t) {
        mRTree->update(mObjects[obj_id], t);
    }

    void deleteObj(const ObjectID& obj_id, const Time& t) {
        mRTree->erase(mObjects[obj_id], t);
    }


    struct CutNode;
    struct Cut;

    //typedef Prox::RTree<SimulationTraits, BoundingSphereData<SimulationTraits, CutNode*>, CutNode*> RTree;
    typedef Prox::RTree<SimulationTraits, MaxSphereData<SimulationTraits, CutNode*>, CutNode*> RTree;
    typedef typename RTree::RTreeNodeType RTreeNodeType;


    struct CutNode {
        Cut* parent;
        RTreeNodeType* rtnode;
        bool satisfies;

        CutNode(Cut* _parent, RTreeNodeType* _rt)
         : parent(_parent),
           rtnode(_rt),
           satisfies(false)
        {
            rtnode->insertCutNode(this);
        }

        ~CutNode() {
            rtnode->eraseCutNode(this);
        }

        bool updateSatisfies(const Vector3& qpos, const BoundingSphere& qregion, float qmaxsize, const SolidAngle& qangle, float qradius) {
            satisfies = rtnode->data().satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
            return satisfies;
        }

        bool leaf() const {
            return rtnode->leaf();
        }

        void handleSplit(RTreeNodeType* orig_node, RTreeNodeType* new_node) {
            parent->handleSplit(this, orig_node, new_node);
        }
        void handleLiftCut(RTreeNodeType* to_node) {
            parent->handleLiftCut(this, to_node);
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
        typedef std::list<CutNode*> CutNodeList;
        typedef typename CutNodeList::iterator CutNodeListIterator;
        typedef typename CutNodeList::const_iterator CutNodeListConstIterator;
        CutNodeList nodes;
        int32 length;

        typedef std::tr1::unordered_set<ObjectID, ObjectIDHasher> ResultSet;
        ResultSet results;

        typedef std::deque<QueryEventType> EventQueue;
        EventQueue events;

        CutNodeListIterator replaceParentWithChildren(const CutNodeListIterator& parent_it) {
            CutNode* parent = *parent_it;
            assert(!parent->leaf());
            // Inserts before, so get next it
            CutNodeListIterator next_it = parent_it;
            next_it++;
            // Insert all new nodes. Going backwards leaves next_it as first of
            // new elements
            for(int i = parent->rtnode->size()-1; i >=0; i--)
                next_it = nodes.insert(next_it, new CutNode(this, parent->rtnode->node(i)));
            // Delete old node
            nodes.erase(parent_it);
            length += (parent->rtnode->size()-1);
            // And clean up
            delete parent;

            return next_it;
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

        void validateCutNodesInRTreeNodes() const {
            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode* node = *it;
                RTreeNodeType* rtnode = node->rtnode;
                assert(rtnode->findCutNode(node) != rtnode->cutNodesEnd());
            }
        }

        void validateCutNodesInTree() const {
            // Get the root base on the first cut node.  Even if this one is
            // broken, we'll be able to tell that the trees have become disjoint
            CutNode* first_cut_node = *(nodes.begin());
            RTreeNodeType* root = _get_root(first_cut_node->rtnode);

            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode* node = *it;
                assert(_is_ancestor(node->rtnode, root));
            }
        };

        // Validates that cut nodes are not through RTree nodes that are
        // ancestors of each other.
        void validateCutNodesUnrelated() const {
            for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode* node = *it;
                for(CutNodeListConstIterator other_it = nodes.begin(); other_it != nodes.end(); other_it++) {
                    CutNode* other_node = *other_it;
                    if (node == other_node) continue;
                    assert( ! _is_ancestor(node->rtnode, other_node->rtnode) );
                    assert( ! _is_ancestor(other_node->rtnode, node->rtnode) );
                }
            }
        };

    public:

        /** Regular constructor.  A new cut simply starts with the root node and
         *  immediately refines.
         */
        Cut(RTreeCutQueryHandler* _parent, QueryType* _query, RTreeNodeType* root)
         : parent(_parent),
           query(_query)
        {
            nodes.push_back(new CutNode(this, root));
            length = 1;
            validateCut();
        }

        ~Cut() {
            for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); it++) {
                CutNode* node = *it;
                delete node;
            }
            nodes.clear();
            length = 0;
        }

        void validateCut() const {
//#define PROXDEBUG
#ifdef PROXDEBUG
            validateCutNodesInRTreeNodes();
            validateCutNodesInTree();
            validateCutNodesUnrelated();
#endif //PROXDEBUG
        };

        void update(LocationServiceCacheType* loc, const Time& t) {
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

            Vector3 qpos = query->position(t);
            BoundingSphere qregion = query->region();
            float qmaxsize = query->maxSize();
            const SolidAngle& qangle = query->angle();
            float qradius = query->radius();

            for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); ) {
                CutNode* node = *it;
                bool last_satisfies = node->satisfies;
                bool satisfies = node->updateSatisfies(qpos, qregion, qmaxsize, qangle, qradius);

                // No matter what, if we don't satisfy the constraints there's
                // no reason to continue with this node.
                if (!satisfies) {
                    it++;
                    continue;
                }

                // What we do with satisfying nodes depends on whether they are
                // a leaf or not
                if (node->leaf()) {
                    // For leaves, we check each child and ensure its result set
                    // membership is correct.
                    for(int i = 0; i < node->rtnode->size(); i++) {
                        ObjectID child_id = loc->iteratorID(node->rtnode->object(i).object);
                        bool child_satisfies = node->rtnode->childData(i, loc, t).satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
                        typename ResultSet::iterator result_it = results.find(child_id);
                        bool in_results = (result_it != results.end());
                        if (child_satisfies && !in_results) {
                            results.insert(child_id);
                            events.push_back(QueryEventType(QueryEventType::Added, child_id));
                        }
                        else if (!child_satisfies && in_results) {
                            results.erase(result_it);
                            events.push_back(QueryEventType(QueryEventType::Removed, child_id));
                        }
                    }

                    it++;
                }
                else {
                    // For internal nodes that satisfy, we replace this node
                    // with the children. Note: don't increment since we need to
                    // start with the first child, which will now be it
                    it = replaceParentWithChildren(it);
                }
            }

            validateCut();

            // FIXME implement pushing cut up where strings of non-satisfying
            // nodes allows

            query->pushEvents(events);
        }

        // Handle a split of orig_node into orig_node and new_node. cnode is the
        // CutNode that was (and remains) at orig_node.
        void handleSplit(CutNode* cnode, RTreeNodeType* orig_node, RTreeNodeType* new_node) {
            // Add a new CutNode to new_node and insert it in our cut list.
            // Future updates will take care of any additional changes (push up
            // or down) that still need to be applied to the tree.

            // FIXME we could avoid this linear search by storing iterators in CutNode
            CutNodeListIterator orig_list_it = std::find(nodes.begin(), nodes.end(), cnode);
            CutNodeListIterator after_orig_list_it = orig_list_it; orig_list_it++;

            nodes.insert(after_orig_list_it, new CutNode(this, new_node));
            length++;

            // Mid-operation, no validation
        }

        void handleLiftCut(CutNode* cnode, RTreeNodeType* to_node) {
            validateCut();

            // This is tricky. The cutnode may be nowhere near the node we need
            // to pull up to. Instead, we have to find all cut nodes whose nodes
            // are children of to_node, destroy them, and replace them with a
            // single cut node at to_node.

            // FIXME if we hit a leaf we need to remove objects from teh result set
            for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); ) {
                CutNode* node = *it;

                if ( _is_ancestor(node->rtnode, to_node) ) {
                    it = nodes.erase(it);
                    delete node;
                }
                else {
                    it++;
                }
            }

            // New node insertion must happen at the end to avoid removing the
            // new node
            // FIXME to preserve ordering we need to select insertion point more carefully
            nodes.insert(nodes.begin(), new CutNode(this, to_node));

            validateCut();
        }

        void handleObjectRemoved(CutNode* cnode, const LocCacheIterator& objit) {
            // We just need to remove the object from the result set if we have
            // it.
            ObjectID child_id = parent->mLocCache->iteratorID(objit);
            typename ResultSet::iterator result_it = results.find(child_id);
            bool in_results = (result_it != results.end());
            if (in_results) {
                results.erase(result_it);
                events.push_back(QueryEventType(QueryEventType::Removed, child_id));
            }

            validateCut();
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

    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, ObjectIDHasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;
    typedef typename QueryMap::const_iterator QueryMapConstIterator;

    LocationServiceCacheType* mLocCache;

    RTree* mRTree;
    ObjectSet mObjects;
    QueryMap mQueries;
    Time mLastTime;
    uint16 mElementsPerNode;
}; // class RTreeCutQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_CUT_QUERY_HANDLER_HPP_
