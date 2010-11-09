/*  libprox
 *  RTreeBulk.hpp
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

#ifndef _PROX_RTREE_BULK_HPP_
#define _PROX_RTREE_BULK_HPP_

#include "Range.hpp"

namespace Prox {

// This is the wrapper for an object we use when bulk loading.  It avoids
// constantly recomputing data.
template<typename SimulationTraits, typename NodeData>
struct BulkLoadElement {
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    LocCacheIterator iterator;
    NodeData data;

    BulkLoadElement()
     : iterator(), data()
    {}
    BulkLoadElement(const LocCacheIterator& _it, const NodeData& _data)
     : iterator(_it), data(_data)
    {}

    struct DimComparator {
        DimComparator(int _dim) : dim(_dim) {}

        bool operator()(const BulkLoadElement& lhs, const BulkLoadElement& rhs) {
            return (lhs.data.getBounds().center()[dim] < rhs.data.getBounds().center()[dim]);
        }

        int dim;
    };
};

template<typename NodeData>
struct SweepData {
    NodeData left;
    NodeData right;
};

template<typename SimulationTraits, typename NodeData, typename CutNode>
class BulkLoadSubTreeInfo {
public:
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    struct SubTreeInfo {
        RTreeNodeType* node;
        float cost;
    };

    std::vector<SubTreeInfo> children;

    BulkLoadSubTreeInfo()
    {}

    void append(RTreeNodeType* node, float cost) {
        SubTreeInfo info;
        info.node = node;
        info.cost = cost;
        children.push_back(info);
    }
    size_t size() const { return children.size(); }
};


// Utility method for RTree_rebuild_build_subtree which takes a list of nodes
// and converts them into a subtree.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_rebuild_build_subtree_from_list(
    const BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode>& subtree_list,
    int branching,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb
)
{
    assert((int)subtree_list.children.size() <= branching);
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    RTreeNodeType* parent = new RTreeNodeType(branching, cb);
    parent->leaf(false);

    for(int i = 0; i < (int)subtree_list.children.size(); i++)
        parent->insert(subtree_list.children[i].node, cb);

    return parent;
}

// Take a range of objects and build a subtree out of them. The returned
// collection is the set of nodes that should be grouped together.  The caller
// should either generate a node for them or continue to aggregate with other
// nodes if that is more efficient.
//
// This essentially breaks our process into two stages. The top down split
// creates a binary tree, although it is only implicit by subdividing the
// objects, reordering, and splitting them.  The second pass takes this binary
// tree and decides how to compress it, combining multiple levels of the binary
// tree into a single RTree node.  This "compression" stops either when we hit
// the branching factor or if the compression would be more expensive than the
// binary tree.
template<typename SimulationTraits, typename NodeData, typename CutNode>
BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> RTree_rebuild_build_subtree(
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    std::vector< BulkLoadElement<SimulationTraits, NodeData> >& objects,
    int branching,
    Range range,
    NodeData parent_data,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb
)
{
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocIterator;
    typedef BulkLoadElement<SimulationTraits, NodeData> BulkLoadElementType;
    typedef std::vector<BulkLoadElementType> ObjectVector;

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // Take the current range and figure out the best splitting point, or maybe
    // choose to just make a leaf node.

    float bestCost = -1.f;
    int bestDim = -1, bestEvent = -1;
    NodeData bestLeftData, bestRightData;
    // Try splitting in each dimension
    for(int dim = 0; dim < 3; dim++) {
        std::sort(objects.begin() + range.start, objects.begin() + range.end + 1, typename BulkLoadElementType::DimComparator(dim));

        std::vector<SweepData<NodeData> > sweep(range.size());

        // Sweep from left to generate left areas
        NodeData leftCurrent;
        for(int i = 0; i < (int)range.size(); i++) {
            assert(objects[i].data.getBounds().center().x > -2.65716055e+37);
            leftCurrent.mergeIn(objects[ range.start + i ].data);
            sweep[i].left = leftCurrent;
        }

        // Sweep from right to generate right areas. Then using the two areas
        // decide whether we have a better split.
        NodeData rightCurrent;
        for(int i = range.size()-1; i > 0; i--) {
            rightCurrent.mergeIn(objects[ range.start + i ].data);
            sweep[i].right = rightCurrent;

            float
                p_left = NodeData::hitProbability(parent_data, sweep[i].left),
                p_right = NodeData::hitProbability(parent_data, sweep[i].right);
            int n_left = i, n_right = (range.size()-i);
            float cost = (2 * COST_NODE_TEST) + (p_left * n_left * COST_LEAF_TEST) + (p_right * n_right * COST_LEAF_TEST);
            if (cost < bestCost || bestCost < 0.f) {
                bestCost = cost;
                bestDim = dim;
                bestEvent = i;
                bestLeftData = sweep[i].left;
                bestRightData = sweep[i].right;
            }
        }
    }


    float no_split_cost = range.size() * COST_LEAF_TEST;
    if ((int)range.size() < branching && (no_split_cost < bestCost || bestCost < 0.f)) {
        // Make a leaf since we can't seem to do any better
        RTreeNodeType* node = new RTreeNodeType(branching, cb);
        node->leaf(true);
        for(typename ObjectVector::iterator it = objects.begin() + range.start; it != objects.begin() + range.end + 1; it++)
            node->insert(loc, it->iterator, t, cb);
        BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> retval;
        retval.append(node, no_split_cost);
        return retval;
    }
    else {
        // Otherwise, we generate children for each side of the split and create
        // a parent node for them.

        assert(bestDim != -1 && bestEvent != -1);
        // We need to get back into the state of the best
        std::sort(objects.begin() + range.start, objects.begin() + range.end + 1, typename BulkLoadElementType::DimComparator(bestDim));

        BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> left_nodes =
            RTree_rebuild_build_subtree<SimulationTraits, NodeData, CutNode>(
                loc, t, objects, branching, Range(range.start, range.start + bestEvent), bestLeftData, cb
            );
        BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> right_nodes =
            RTree_rebuild_build_subtree<SimulationTraits, NodeData, CutNode>(
                loc, t, objects, branching, Range(range.start + bestEvent + 1, range.end), bestRightData, cb
            );


        // Compute the cost if we handle the elements when split up (i.e.,
        // maintain the binary tree at this level) and if we handle them
        // together (i.e. we merge the two groups into one group handled by this
        // parent). Note that it is still safe to use bestLeftData and
        // bestRightData since the split there is still valid.

        // Direct (merged) test cost: visiting this node results in immediately
        // testing all the given nodes
        float merged_cost = (float)(left_nodes.size() + right_nodes.size()) * COST_NODE_TEST;
        // Indirect (split, keeping hierarchy) test cost: visiting this node
        // results in immediately visiting two children and, depending on the
        // outcome there, possibly visiting the left and/or right children nodes
        float split_cost = 2.f * COST_NODE_TEST;
        float p_split_left = NodeData::hitProbability(parent_data, bestLeftData),
            p_split_right = NodeData::hitProbability(parent_data, bestRightData);
        float left_split_cost = 0.f;
        for(int i = 0; i < (int)left_nodes.size(); i++) {
            merged_cost += NodeData::hitProbability(parent_data, left_nodes.children[i].node->data()) * left_nodes.children[i].cost;
            left_split_cost += NodeData::hitProbability(bestLeftData, left_nodes.children[i].node->data()) * left_nodes.children[i].cost;
        }
        split_cost += p_split_left * left_split_cost;
        float right_split_cost = 0.f;
        for(int i = 0; i < (int)right_nodes.size(); i++) {
            merged_cost += NodeData::hitProbability(parent_data, right_nodes.children[i].node->data()) * right_nodes.children[i].cost;
            right_split_cost += NodeData::hitProbability(bestRightData, right_nodes.children[i].node->data()) * right_nodes.children[i].cost;
        }
        split_cost += p_split_right * right_split_cost;

        // If we can do better by handling the nodes we got back directly, then
        // do it
        if (
            (int)(left_nodes.size() + right_nodes.size()) <= branching &&
            merged_cost < split_cost
        ) {
            BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> retval;
            for(int i = 0; i < (int)right_nodes.size(); i++)
                retval.append(right_nodes.children[i].node, right_nodes.children[i].cost);
            for(int i = 0; i < (int)left_nodes.size(); i++)
                retval.append(left_nodes.children[i].node, left_nodes.children[i].cost);
            return retval;
        }
        else { // Otherwise, build nodes out of them and continue
            BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> retval;
            retval.append( RTree_rebuild_build_subtree_from_list(left_nodes, branching, cb), left_split_cost );
            retval.append( RTree_rebuild_build_subtree_from_list(right_nodes, branching, cb), right_split_cost );
            return retval;
        }
    }
}


// The full rebuild process, just a driver for helper methods that do the actual
// splitting and generation of nodes.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_rebuild(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const std::vector<typename LocationServiceCache<SimulationTraits>::Iterator>& object_iterators,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb
)
{
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename RTreeNodeType::Index Index;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef BulkLoadElement<SimulationTraits, NodeData> BulkLoadElementType;
    typedef std::vector<BulkLoadElementType> ObjectVector;

    // Use the same branching factor
    Index branching = root->capacity();

    // Collect all the information we need about the objects, specifying that we
    // are now tracking them and getting our own iterators to them. This will
    // ensure they don't get released in the transition.
    ObjectVector objects(object_iterators.size());
    NodeData root_data;
    int idx = 0;
    for(int i = 0;  i < object_iterators.size(); i++) {
        LocCacheIterator loc_it = object_iterators[i];
        ObjectID objid = loc->iteratorID(loc_it);
        objects[idx] = BulkLoadElementType(loc_it, NodeData(loc, loc_it, t));
        root_data.mergeIn(objects[idx].data);
        assert(objects[idx].data.getBounds().center().x > -2.65716055e+37);
        idx++;
    }

    BulkLoadSubTreeInfo<SimulationTraits, NodeData, CutNode> root_children =
        RTree_rebuild_build_subtree<SimulationTraits, NodeData, CutNode>(loc, t, objects, branching, Range(object_iterators.size()), root_data, cb);
    return RTree_rebuild_build_subtree_from_list(root_children, branching, cb);
}

} // namespace Prox

#endif //_PROX_RTREE_BULK_HPP_
