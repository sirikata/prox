/*  libprox
 *  RTree.hpp
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

#ifndef _PROX_RTREE_HPP_
#define _PROX_RTREE_HPP_

#include <prox/util/Platform.hpp>
#include <prox/base/LocationServiceCache.hpp>
#include <prox/rtree/Constraints.hpp>
#include <prox/base/Aggregator.hpp>
#include <prox/base/AggregateListener.hpp>
#include <float.h>

#include "RTreeCore.hpp"
#include "RTreeCost.hpp"
#include "RTreeRestructure.hpp"
#include "RTreeBulk.hpp"

namespace Prox {

/**************************************************************************
 * RTree - Wrapper for an RTreeNode root node to provide a nicer interface
 **************************************************************************/
template<typename SimulationTraits, typename NodeData, typename CutNode>
class RTree {
public:
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef NodeData NodeDataType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef typename SimulationTraits::ObjectIDNullType ObjectIDNull;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    typedef Aggregator<SimulationTraits> AggregatorType;
    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef typename std::tr1::function<bool()> ReportRestructuresCallback;

    typedef typename std::tr1::function<void()> RootCreatedCallback;
    typedef typename RTreeNodeType::RootReplacedByChildCallback RootReplacedByChildCallback;
    typedef typename RTreeNodeType::NodeSplitCallback NodeSplitCallback;

    typedef typename RTreeNodeType::LiftCutCallback LiftCutCallback;
    typedef typename RTreeNodeType::ReorderCutCallback ReorderCutCallback;

    typedef typename RTreeNodeType::ObjectInsertedCallback ObjectInsertedCallback;
    typedef typename RTreeNodeType::ObjectRemovedCallback ObjectRemovedCallback;

    typedef typename RTreeNodeType::NodeAddedAboveCutCallback NodeAddedAboveCutCallback;
    typedef typename RTreeNodeType::NodeRemovedCallback NodeRemovedCallback;

    typedef typename RTreeNodeType::NodeReparentedCallback NodeReparentedCallback;
    typedef typename RTreeNodeType::ObjectReparentedCallback ObjectReparentedCallback;

    typedef typename RTreeNodeType::Index Index;

    RTree(Index elements_per_node, LocationServiceCacheType* loccache,
        bool static_objects, bool replicated,
        ReportRestructuresCallback report_restructures_cb,
        RootCreatedCallback root_created_cb,
        AggregatorType* aggregator = NULL,
        AggregateListenerType* agg = NULL,
        RootReplacedByChildCallback root_replaced_cb = 0,
        RootReplacedByChildCallback replicated_root_created_cb = 0,
        RootReplacedByChildCallback replicated_root_destroyed_cb = 0,
        NodeSplitCallback node_split_cb = 0,
        NodeSplitCallback node_split_finished_cb = 0,
        NodeSplitCallback replicated_node_split_cb = 0,
        LiftCutCallback lift_cut_cb = 0, ReorderCutCallback reorder_cut_cb = 0,
        ObjectInsertedCallback obj_ins_cb = 0, ObjectRemovedCallback obj_rem_cb = 0,
        NodeAddedAboveCutCallback node_add_above_cb = 0,
        NodeRemovedCallback node_rem_cb = 0,
        NodeReparentedCallback node_reparent_cb = 0,
        NodeReparentedCallback node_reparent_above_cb = 0,
        ObjectReparentedCallback object_reparent_cb = 0
    )
     : mLocCache(loccache),
       mElementsPerNode(elements_per_node),
       mStaticObjects(static_objects),
       mReplicated(replicated),
       mRestructureMightHaveEffect(false),
       mReportRestructures(report_restructures_cb)
    {
        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;

        mCallbacks.aggregator = aggregator;
        mCallbacks.aggregate = agg;
        mCallbacks.objectLeafChanged = std::tr1::bind(&RTree::onObjectLeafChanged, this, _1, _2);
        mCallbacks.getObjectLeaf = std::tr1::bind(&RTree::getObjectLeaf, this, _1);
        mCallbacks.rootReplaced = root_replaced_cb;
        mCallbacks.replicatedRootCreated = replicated_root_created_cb;
        mCallbacks.replicatedRootDestroyed = replicated_root_destroyed_cb;
        mCallbacks.nodeSplit = node_split_cb;
        mCallbacks.nodeSplitFinished = node_split_finished_cb;
        mCallbacks.replicatedNodeSplit = replicated_node_split_cb;
        mCallbacks.liftCut = lift_cut_cb;
        mCallbacks.reorderCut = reorder_cut_cb;
        mCallbacks.objectInserted = obj_ins_cb;
        mCallbacks.objectRemoved = obj_rem_cb;
        mCallbacks.nodeAddedAboveCut = node_add_above_cb;
        mCallbacks.nodeWithCutRemoved = node_rem_cb;
        mCallbacks.nodeReparented = node_reparent_cb;
        mCallbacks.nodeReparentedAboveCut = node_reparent_above_cb;
        mCallbacks.objectReparented = object_reparent_cb;
        mRootCreatedCallback = root_created_cb;

        // If we're replicating a tree, we don't want our own root, we just want
        // to create root(s) as requested by the incoming update stream
        mRoot = NULL;
    }

    ~RTree() {
        if (mRoot != NULL) RTree_destroy_tree(mRoot);
    }


    bool staticObjects() const { return mStaticObjects; }
    typename RTreeNodeType::Index elementsPerNode() const { return mElementsPerNode; }
    bool replicated() const { return mReplicated; }
    LocationServiceCacheType* loc() const { return mLocCache; }
    const typename RTreeNodeType::Callbacks& callbacks() const { return mCallbacks; };

    RTreeNodeType* root() {
        return mRoot;
    }
    const RTreeNodeType* root() const {
        return mRoot;
    }

    int size() const { return mRoot != NULL ? mRoot->treeSize() : 0; }

    void insert(const LocCacheIterator& obj, const Time& t) {
        assert(!replicated());

        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) == mObjectLeaves.end());
        mObjectLeaves[objid] = NULL;

        ensureHaveRoot();
        mRoot = RTree_insert_object(mRoot, obj, t);

        mRestructureMightHaveEffect = true;
    }

    void insert(const LocCacheIterator& obj, const ObjectID& parent, const Time& t) {
        assert(replicated());

        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) == mObjectLeaves.end());
        mObjectLeaves[objid] = NULL;

        assert(mRTreeNodes.find(parent) != mRTreeNodes.end());
        RTreeNodeType* at_node = mRTreeNodes[parent];
        mRoot = RTree_insert_object_at_node(at_node, obj, t);

        mRestructureMightHaveEffect = true;
    }

    void insertNode(const LocCacheIterator& node, const ObjectID& parent, const Time& t) {
        assert(replicated());
        const ObjectID& nodeid = mLocCache->iteratorID(node);
        assert(mRTreeNodes.find(nodeid) == mRTreeNodes.end());

        assert(parent == ObjectIDNull()() || mRTreeNodes.find(parent) != mRTreeNodes.end());

        RTreeNodeType* new_node = RTree_create_new_node<SimulationTraits, NodeData, CutNode>(this, node, t);
        mRTreeNodes[nodeid] = new_node;

        if (parent == ObjectIDNull()()) {
            // This is a root node. We might already have a root node
            // that we're just putting a new parent on, or it's just
            // the first node we've seen replicated
            // TODO(ewencp) when we deal with rebuilding trees getting
            // replicated, this won't be sufficient...
            if (mRoot != NULL)
                RTree_prepend_new_root(new_node, mRoot);
            mRoot = new_node;
            if (mRootCreatedCallback != 0) mRootCreatedCallback();
        }
        else {
            RTreeNodeType* at_node = mRTreeNodes[parent];
            RTree_insert_new_node_at_node(new_node, at_node, t);
        }
    }

    void update(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_update_object(mRoot, objid, t);

        mRestructureMightHaveEffect = true;
    }

    void updateNode(const LocCacheIterator& node_locit, const Time& t) {
        assert(replicated());
        const ObjectID& objid = mLocCache->iteratorID(node_locit);
        assert(mRTreeNodes.find(objid) != mRTreeNodes.end());

        RTree_update_node(mRTreeNodes[objid], node_locit, t);
    }

    void update(const Time& t) {
        if (!mStaticObjects && mRoot != NULL)
            mRoot = RTree_update_tree(mRoot, t);
    }


    void reparentObject(const LocCacheIterator& obj, const ObjectID& parentid, const Time& t) {
        // Currently taking the easy way out of this -- just perform a
        // removal and then addition.
        erase(obj, t, true);
        insert(obj, parentid, t);
    }

    void reparentNode(const LocCacheIterator& node, const ObjectID& parentid, const Time& t) {
        assert(replicated());

        const ObjectID& nodeid = mLocCache->iteratorID(node);
        typename ObjectIDNodeMap::const_iterator nodeit = mRTreeNodes.find(nodeid);
        assert(nodeit != mRTreeNodes.end());
        RTreeNodeType* rtnode = nodeit->second;

        typename ObjectIDNodeMap::const_iterator parentit = mRTreeNodes.find(parentid);
        assert(parentit != mRTreeNodes.end());
        RTreeNodeType* parent_rtnode = parentit->second;

        RTree_reparent_node(rtnode, parent_rtnode, t);

        mRestructureMightHaveEffect = true;
    }


    void reportBounds(const Time& t) {
        RTree_report_bounds(stdout, mRoot, t);
        fprintf(stdout, "\n");
    }

    void restructure(const Time& t) {
        // Only restructure if we're dealing with dynamic objects or if
        // something has changed/the last restructure pass did something
        if (!mStaticObjects || mRestructureMightHaveEffect) {
            RestructureInfo info = RTree_restructure_tree(mRoot, t);
            if (mReportRestructures && mReportRestructures())
                printf("{ \"time\" : %d, \"count\" : %d, \"cuts-rebuilt\" : %d }\n", (int)(t-Time::null()).milliseconds(), info.restructures, info.cutRebuilds);
            // Without any additions/removals, a restructure pass can only have
            // an effect if this previous pass actually restructured something.
            mRestructureMightHaveEffect = (info.restructures > 0);
        }
    }

    void erase(const LocCacheIterator& obj, const Time& t, bool temporary) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_delete_object(mRoot, obj, t, temporary);
        mObjectLeaves.erase(objid);

        mRestructureMightHaveEffect = true;
    }

    void eraseNode(const LocCacheIterator& node, const Time& t, bool temporary) {
        assert(replicated());

        const ObjectID& nodeid = mLocCache->iteratorID(node);
        typename ObjectIDNodeMap::const_iterator nodeit = mRTreeNodes.find(nodeid);
        assert(nodeit != mRTreeNodes.end());
        RTreeNodeType* rtnode = nodeit->second;
        mRoot = RTree_delete_node(mRoot, rtnode, t, temporary);
        mRTreeNodes.erase(nodeid);

        mRestructureMightHaveEffect = true;
    }

    /** If in debug mode, verify the constraints on the data structure. */
    void verifyConstraints(const Time& t) {
        if (mRoot != NULL) RTree_verify_constraints(mRoot, t);
    }

    void bulkLoad(const std::vector<LocCacheIterator>& object_iterators, const Time& t) {
        assert(!replicated());

        // Bulk loading requires we initialize as with inserting
        for(typename std::vector<LocCacheIterator>::const_iterator objit = object_iterators.begin(); objit != object_iterators.end(); objit++)
            mObjectLeaves[mLocCache->iteratorID(*objit)] = NULL;

        // Then do the actual computation
        ensureHaveRoot();
        mRoot = RTree_rebuild(
            mRoot, t,
            object_iterators
        );
        mRestructureMightHaveEffect = true;
    }

    float cost(const Time& t) {
        return (mRoot != NULL) ? RTree_cost(mRoot, t) : 0.f;
    }


    void draw() {
        RTree_draw_tree(mRoot);
    }

    // Iteration over nodes. This just means internal nodes are visited as
    // well. This iterator does pre-order traversal. This is most convenient if
    // you're trying to replicate the tree since it gives you the tree top-down,
    // i.e. you always have the parents before getting their children.
    class NodeIteratorBase {
    public:
        NodeIteratorBase(RTree* o, RTreeNodeType* r)
         : owner(o), node(r), idx(-1)
        {}

        // true if this points at a valid value, false if it's end()
        bool valid() const {
            return (
                node != NULL &&
                (idx == -1 || idx < node->size())
            );
        }

        NodeIteratorBase& operator++() { //prefix
            assert(valid());

            // Move the child index along. For internal nodes this'll push us
            // past self-processing and move to children, for leaf nodes it'll
            // move us to the next child or over the end of the list.
            idx++;

            // Our traversal only uses internal nodes as node once: when they
            // visit them. They then start working on children. We should always
            // be hitting the first child here.
            if (!node->objectChildren()) {
                assert(idx == 0);
                // If we have a replicated tree, internal nodes could
                // be empty. Make sure we handle that by skipping
                // children if we have none.
                if (!node->empty()) {
                    node = node->node(idx);
                    idx = -1;
                    return *this;
                }
            }

            // Otherwise, we're processing a leaf node. The simple case is if we
            // just have another child to process
            if (idx < node->size())
                return *this;

            // Otherwise we've hit the end of this node and need to move onto
            // the parent's next node. However, this process can chain upwards
            // -- we may have reached the end of the parent, and the end of the
            // grandparent, etc.
            RTreeNodeType* parent = node->parent();
            do {
                // If we've hit the end of everything, set ourselves to end().
                if (parent == NULL) {
                    node = NULL;
                    idx = -1;
                    return *this;
                }

                // Otherwise, find ourselves in the parent
                Index par_idx;
                for(par_idx = 0; par_idx < parent->size(); par_idx++)
                    if (parent->node(par_idx) == node) break;
                assert(par_idx < parent->size());
                // And move it forward
                par_idx++;
                // If the index is still a valid child, we can move onto that
                // child
                if (par_idx < parent->size()) {
                    node = parent->node(par_idx);
                    idx = -1;
                    return *this;
                }
                // Otherwise, we've gone over the end of the parent too. Move us
                // up to the parent, the parent to the grandparent, and keep
                // working our way up.
                node = parent;
                parent = parent->parent();
                // Do nothing to idx. It isn't used in this loop and will get
                // reset on either of the possible exits.
            } while(true);
            // We don't really need this except to satisfy the compiler: we can
            // only end the above loop in two ways, finding another node in one
            // of the ancestors or hitting the root with no more work.
            return *this;
        }

        NodeIteratorBase operator++(int) { //postfix
            NodeIteratorBase orig = *this;
            ++(*this);
            return orig;
        }

        const ObjectID& id() const {
            assert(valid());
            // If we haven't processed the current node yet
            if (idx == -1)
                return node->aggregateID();

            // If we're processing children, we should be at a leaf node
            assert(node->objectChildren());
            return owner->mLocCache->iteratorID(node->object(idx).object);
        }

        ObjectID parentId() const {
            assert(valid());
            // If we haven't processed the current node yet
            if (idx == -1) {
                if (node->parent() != NULL)
                    return node->parent()->aggregateID();
                else
                    return ObjectIDNull()();
            }

            // If we're processing children, we should be at a leaf node
            assert(node->objectChildren());
            return node->aggregateID();
        }
        BoundingSphere bounds(const Time& t) const {
            assert(valid());
            // If we haven't processed the current node yet
            if (idx == -1)
                return node->data().getBounds();

            // If we're processing children, we should be at a leaf node
            assert(node->objectChildren());
            return owner->mLocCache->worldCompleteBounds(node->object(idx).object, t);
        }
        uint32 cuts() const {
            assert(valid());
            // If we haven't processed the current node yet
            if (idx == -1)
                return node->cutNodesSize();

            // If we're processing children, we should be at a leaf node. Cuts
            // don't go through objects even though they are presented as nodes
            assert(node->objectChildren());
            return 0;
        }

        bool operator==(const NodeIteratorBase& rhs) {
            return (node == rhs.node && (node == NULL || idx == rhs.idx));
        }
        bool operator!=(const NodeIteratorBase& rhs) {
            return !(*this == rhs);
        }

    private:
        // Parent RTree
        RTree* owner;
        // Current node being processed
        RTreeNodeType* node;
        // Child index in this node, or -1 if we we're still processing this
        // node. The index only applies for leaf nodes as node, rather than idx,
        // will change as we traverse children nodes. When we finish a child
        // node, we figure out the next child to process by finding the index of
        // the child we just finished with.
        int16 idx;
    };
    typedef NodeIteratorBase NodeIterator;

    NodeIterator nodesBegin() { return NodeIterator(this, mRoot); }
    NodeIterator nodesEnd() { return NodeIterator(this, NULL); }

private:
    // Ensure we have a root node. Should only be used for non-replicating-tree
    // operations since this will create missing root nodes, which would be
    // incorrect for replication.
    void ensureHaveRoot() {
        if (mRoot == NULL) {
            mRoot = new RTreeNodeType(this);
            if (mRootCreatedCallback != 0) mRootCreatedCallback();
        }
    }

    typedef std::tr1::unordered_map<ObjectID, RTreeNodeType*, ObjectIDHasher> ObjectIDNodeMap;
    typedef ObjectIDNodeMap ObjectLeafIndex;

    // Iteration over objects
    template<class InternalIterator>
    class ObjectIDIteratorWrapper {
    public:
        ObjectIDIteratorWrapper(InternalIterator _it)
         :it(_it)
        {}

        ObjectIDIteratorWrapper& operator++() {
            it++;
            return *this;
        }
        ObjectIDIteratorWrapper& operator++(int) {
            it++;
            return *this;
        }

        const ObjectID& id() const { return it->first; }
        const ObjectID& operator*() const { return it->first; }

        bool operator==(const ObjectIDIteratorWrapper& rhs) {
            return it == rhs.it;
        }
        bool operator!=(const ObjectIDIteratorWrapper& rhs) {
            return it != rhs.it;
        }
    private:
        InternalIterator it;
    };
    typedef ObjectIDIteratorWrapper<typename ObjectLeafIndex::iterator> ObjectIDIterator;
    typedef ObjectIDIteratorWrapper<typename ObjectLeafIndex::const_iterator> ConstObjectIDIterator;

    ObjectIDIterator objectsBegin() { return ObjectIDIterator(mObjectLeaves.begin()); }
    ObjectIDIterator objectsEnd() { return ObjectIDIterator(mObjectLeaves.end()); }
    int objectsSize() { return mObjectLeaves.size(); }


    void onObjectLeafChanged(const LocCacheIterator& obj, RTreeNodeType* node) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());
        assert(node != NULL);
        mObjectLeaves[objid] = node;
    }

    RTreeNodeType* getObjectLeaf(const ObjectID& objid) {
        typename ObjectLeafIndex::const_iterator it = mObjectLeaves.find(objid);
        if (it == mObjectLeaves.end()) return NULL;
        return it->second;
    }

    LocationServiceCacheType* mLocCache;
    Index mElementsPerNode;
    RTreeNodeType* mRoot;
    typename RTreeNodeType::Callbacks mCallbacks;
    ObjectLeafIndex mObjectLeaves;
    ObjectIDNodeMap mRTreeNodes;
    bool mStaticObjects;
    const bool mReplicated;
    bool mRestructureMightHaveEffect;
    ReportRestructuresCallback mReportRestructures;
    RootCreatedCallback mRootCreatedCallback;
}; // class RTree

} // namespace Prox

#endif //_PROX_RTREE_HPP_
