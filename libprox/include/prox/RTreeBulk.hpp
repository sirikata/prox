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

#define COST_NODE_TEST 1.f
#define COST_LEAF_TEST 1.f

// Take a range of objects and build a subtree out of them
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ObjectIDIterator>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_rebuild_build_subtree(
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
        return node;
    }
    else {
        // Otherwise, we generate children for each side of the split and create
        // a parent node for them.

        assert(bestDim != -1 && bestEvent != -1);
        // We need to get back into the state of the best
        std::sort(objects.begin() + range.start, objects.begin() + range.end + 1, typename BulkLoadElementType::DimComparator(bestDim));

        RTreeNodeType* left_tree =
            RTree_rebuild_build_subtree<SimulationTraits, NodeData, CutNode, ObjectIDIterator>(
                loc, t, objects, branching, Range(range.start, range.start + bestEvent), bestLeftData, cb
            );
        RTreeNodeType* right_tree =
            RTree_rebuild_build_subtree<SimulationTraits, NodeData, CutNode, ObjectIDIterator>(
                loc, t, objects, branching, Range(range.start + bestEvent + 1, range.end), bestRightData, cb
            );

        RTreeNodeType* parent = new RTreeNodeType(branching, cb);
        parent->leaf(false);
        parent->insert(left_tree, cb);
        parent->insert(right_tree, cb);

        return parent;
    }
}


// The full rebuild process, just a driver for helper methods that do the actual
// splitting and generation of nodes.
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ObjectIDIterator>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_rebuild(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    ObjectIDIterator objects_begin, ObjectIDIterator objects_end, int nobjects,
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
    ObjectVector objects(nobjects);
    NodeData root_data;
    int idx = 0;
    for(ObjectIDIterator objit = objects_begin; objit != objects_end; objit++) {
        ObjectID objid = *objit;
        LocCacheIterator loc_it = loc->startTracking(objid);
        objects[idx] = BulkLoadElementType(loc_it, NodeData(loc, loc_it, t));
        root_data.mergeIn(objects[idx].data);
        assert(objects[idx].data.getBounds().center().x > -2.65716055e+37);
        idx++;
    }

    return RTree_rebuild_build_subtree<SimulationTraits, NodeData, CutNode, ObjectIDIterator>(loc, t, objects, branching, Range(nobjects), root_data, cb);
}

} // namespace Prox

#endif //_PROX_RTREE_BULK_HPP_
