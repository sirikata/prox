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

    typedef typename RTreeNodeType::RootReplacedByChildCallback RootReplacedByChildCallback;
    typedef typename RTreeNodeType::NodeSplitCallback NodeSplitCallback;

    typedef typename RTreeNodeType::LiftCutCallback LiftCutCallback;
    typedef typename RTreeNodeType::ReorderCutCallback ReorderCutCallback;

    typedef typename RTreeNodeType::ObjectInsertedCallback ObjectInsertedCallback;
    typedef typename RTreeNodeType::ObjectRemovedCallback ObjectRemovedCallback;

    typedef typename RTreeNodeType::Index Index;

    RTree(Index elements_per_node, LocationServiceCacheType* loccache,
        bool static_objects,
        ReportRestructuresCallback report_restructures_cb,
        AggregatorType* aggregator = NULL,
        AggregateListenerType* agg = NULL,
        RootReplacedByChildCallback root_replaced_cb = 0, NodeSplitCallback node_split_cb = 0,
        LiftCutCallback lift_cut_cb = 0, ReorderCutCallback reorder_cut_cb = 0,
        ObjectInsertedCallback obj_ins_cb = 0, ObjectRemovedCallback obj_rem_cb = 0
    )
     : mElementsPerNode(elements_per_node),
       mLocCache(loccache),
       mStaticObjects(static_objects),
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
        mCallbacks.nodeSplit = node_split_cb;
        mCallbacks.liftCut = lift_cut_cb;
        mCallbacks.reorderCut = reorder_cut_cb;
        mCallbacks.objectInserted = obj_ins_cb;
        mCallbacks.objectRemoved = obj_rem_cb;

        mRoot = NULL;
    }

    // Split into constructor + initialize because callbacks in the constructor
    // can cause problems (e.g. caller not having the pointer back to the RTree
    // yet).
    void initialize() {
        mRoot = new RTreeNodeType(mElementsPerNode, mCallbacks);
    }

    ~RTree() {
        RTree_destroy_tree(mRoot, mLocCache, mCallbacks);
    }


    bool staticObjects() const { return mStaticObjects; }


    RTreeNodeType* root() {
        return mRoot;
    }
    const RTreeNodeType* root() const {
        return mRoot;
    }

    int size() const { return mRoot->treeSize(); }

    void insert(const LocCacheIterator& obj, const Time& t) {
       
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) == mObjectLeaves.end());
        mObjectLeaves[objid] = NULL;

        mRoot = RTree_insert_object(mRoot, mLocCache, obj, t, mCallbacks);

        mRestructureMightHaveEffect = true;                
    }

    void update(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_update_object(mRoot, mLocCache, objid, t, mCallbacks);

        mRestructureMightHaveEffect = true;
    }

    void update(const Time& t) {
        if (!mStaticObjects)
            mRoot = RTree_update_tree(mRoot, mLocCache, t, mCallbacks);
    }

    void reportBounds(const Time& t) {
        RTree_report_bounds(stdout, mRoot, mLocCache, t);
        fprintf(stdout, "\n");
    }

    void restructure(const Time& t) {
        // Only restructure if we're dealing with dynamic objects or if
        // something has changed/the last restructure pass did something
        if (!mStaticObjects || mRestructureMightHaveEffect) {
            RestructureInfo info = RTree_restructure_tree(mRoot, mLocCache, t, mCallbacks);
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

        mRoot = RTree_delete_object(mRoot, mLocCache, obj, t, temporary, mCallbacks);
        mObjectLeaves.erase(objid);

        mRestructureMightHaveEffect = true;
    }

    /** If in debug mode, verify the constraints on the data structure. */
    void verifyConstraints(const Time& t) {
        RTree_verify_constraints(mRoot, mLocCache, t);
    }

    void bulkLoad(const std::vector<LocCacheIterator>& object_iterators, const Time& t) {
        // Bulk loading requires we initialize as with inserting
        for(typename std::vector<LocCacheIterator>::const_iterator objit = object_iterators.begin(); objit != object_iterators.end(); objit++)
            mObjectLeaves[mLocCache->iteratorID(*objit)] = NULL;

        // Then do the actual computation
        mRoot = RTree_rebuild(
            mRoot, mLocCache, t,
            object_iterators,
            mCallbacks
        );
        mRestructureMightHaveEffect = true;
    }

    float cost(const Time& t) {
        return RTree_cost(mRoot, mLocCache, t);
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
            if (!node->leaf()) {
                assert(idx == 0);
                assert(node->size() > 0); // Non-leaf nodes should always have
                                          // some children
                node = node->node(idx);
                idx = -1;
                return *this;
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
            assert(node->leaf());
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
            assert(node->leaf());
            return node->aggregateID();
        }
        BoundingSphere bounds(const Time& t) const {
            assert(valid());
            // If we haven't processed the current node yet
            if (idx == -1)
                return node->data().getBounds();

            // If we're processing children, we should be at a leaf node
            assert(node->leaf());
            return owner->mLocCache->worldCompleteBounds(node->object(idx).object, t);
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
    typedef std::tr1::unordered_map<ObjectID, RTreeNodeType*, ObjectIDHasher> ObjectLeafIndex;

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
    bool mStaticObjects;
    bool mRestructureMightHaveEffect;
    ReportRestructuresCallback mReportRestructures;
}; // class RTree

} // namespace Prox

#endif //_PROX_RTREE_HPP_
