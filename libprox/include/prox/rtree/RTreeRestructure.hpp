/*  libprox
 *  RTreeRestructure.hpp
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

#ifndef _PROX_RTREE_RESTRUCTURE_HPP_
#define _PROX_RTREE_RESTRUCTURE_HPP_

#include "Range.hpp"

namespace Prox {

template<typename ChildType, typename NodeData>
struct ChildInfo {
    ChildType child;
    NodeData data;

    ChildInfo(const ChildType& _c, const NodeData& _d)
     : child(_c), data(_d)
    {
    }


    struct XComparator {
        bool operator()(const ChildInfo& lhs, const ChildInfo& rhs) { return lhs.data.getBounds().center().x < rhs.data.getBounds().center().x; };
    };
    struct YComparator {
        bool operator()(const ChildInfo& lhs, const ChildInfo& rhs) { return lhs.data.getBounds().center().y < rhs.data.getBounds().center().y; };
    };
    struct ZComparator {
        bool operator()(const ChildInfo& lhs, const ChildInfo& rhs) { return lhs.data.getBounds().center().z < rhs.data.getBounds().center().z; };
    };
};


/** Performs a binary split on a set of children along an axis.  This
 *  is used as a step in reorganizing a set of nodes.  A bunch of
 *  these split_children operations are performed recursively until
 *  the set maps onto a single child node and can be stored.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_restructure_split_children(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb,
    std::vector<ChildInfo<ChildType, NodeData> >& children,
    uint32 child_begin,
    uint32 child_end)
{
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    typedef ChildInfo<ChildType, NodeData> ChildInfoType;
    typedef std::vector<ChildInfoType> ChildInfoList;

    // Find the right axis to split along
    float minx = FLT_MAX, maxx = -FLT_MAX;
    float miny = FLT_MAX, maxy = -FLT_MAX;
    float minz = FLT_MAX, maxz = -FLT_MAX;
    for(uint32 i = child_begin; i <= child_end; i++) {
        BoundingSphere cdata_bounds = children[i].data.getBounds();
        Vector3 cdata_rad3(cdata_bounds.radius(), cdata_bounds.radius(), cdata_bounds.radius());
        Vector3 cdata_min = cdata_bounds.center() - cdata_rad3;
        Vector3 cdata_max = cdata_bounds.center() + cdata_rad3;
        minx = std::min(minx, cdata_min.x);
        maxx = std::max(maxx, cdata_max.x);
        miny = std::min(miny, cdata_min.y);
        maxy = std::max(maxy, cdata_max.y);
        minz = std::min(minz, cdata_min.z);
        maxz = std::max(maxz, cdata_max.z);
    }

    // Sort based on largest dimension
    float diffx = maxx - minx, diffy = maxy - miny, diffz = maxz - minz;
    float maxdiff = std::max(diffx, std::max(diffy, diffz));
    if (maxdiff == diffx)
        std::sort(children.begin() + child_begin, children.begin() + child_end + 1, typename ChildInfoType::XComparator());
    else if (maxdiff == diffy)
        std::sort(children.begin() + child_begin, children.begin() + child_end + 1, typename ChildInfoType::YComparator());
    else if (maxdiff == diffz)
        std::sort(children.begin() + child_begin, children.begin() + child_end + 1, typename ChildInfoType::ZComparator());
}

/** Given a node which has children that are significantly overlapping,
 *  restructures the grandchildren of the node to improve the children's
 *  layout.  Returns number of cuts affected by this process.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_restructure_nodes_children(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb,
    std::tr1::unordered_set<typename CutNode::CutType*>* affected_cuts)
{
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    // We collect a list of all the cuts that will be affected by the
    // reorganization.  This will boil down to any cuts that pass through any
    // children or descendents of the children.  After reorganization there
    // could be violations in cut nodes, so the more complex rebuilding process,
    // handling gaps and overlaps, must be used.
    for(int i = 0; i < node->size(); i++) {
        RTreeNodeType* child_node = node->node(i);
        RTree_collect_cuts(child_node, affected_cuts);
    }


    // Next we need to collect all the information about the grandchildren we're rearranging
    ChildOperations child_ops;

    typedef ChildInfo<ChildType, NodeData> ChildInfoType;
    std::vector<ChildInfoType> split_children;

    // Add all grandchildren and clear out the child nodes.
    for(int i = 0; i < node->size(); i++) {
        RTreeNodeType* child_node = node->node(i);
        for(int j = 0; j < child_node->size(); j++)
            split_children.push_back( ChildInfoType( child_ops.childData(child_node, j), child_node->childData(j,loc,t) ) );
        child_node->clear(loc, cb);
    }

    //printf("Restructuring %d %d\n", (int)node->size(), (int)split_children.size());

    // "Recursively" split subgroups of children.
    std::stack<RangeMapping> childRangesStack;
    // Start with the full range for both children and grandchildren
    childRangesStack.push(
        RangeMapping(node->size(), split_children.size(), 0, node->size()-1)
    );
    while(!childRangesStack.empty()) {
        RangeMapping ranges = childRangesStack.top();
        childRangesStack.pop();

        // Base case - we've reached a single child
        if (ranges.singleDiv()) {
            // Map the children in the range back into the child
            for(uint32 i = ranges.grandchildStart(); i <= ranges.grandchildEnd(); i++)
                child_ops.insert( node->node(ranges.child()), loc, split_children[i].child, t, cb);
        }
        else {
            RTree_restructure_split_children<SimulationTraits, NodeData, CutNode, ChildType, ChildOperations>(
                node, loc, t, cb, split_children, ranges.grandchildStart(), ranges.grandchildEnd()
            );
            childRangesStack.push(ranges.bottomHalf());
            childRangesStack.push(ranges.topHalf());
        }
    }
}

struct RestructureInfo {
    RestructureInfo()
     : restructures(0), cutRebuilds(0)
    {}
    RestructureInfo(int res, int rebuilds)
     : restructures(res), cutRebuilds(rebuilds)
    {}

    RestructureInfo operator+(const RestructureInfo& rhs) const {
        return RestructureInfo(
            restructures + rhs.restructures,
            cutRebuilds + rhs.cutRebuilds
        );
    }

    void operator+=(const RestructureInfo& rhs) {
        restructures += rhs.restructures;
        cutRebuilds += rhs.cutRebuilds;
    }

    int restructures;
    int cutRebuilds;
};

/* Recursively restructure the tree by looking for nodes with children that have
 * become inefficient and restructuring them. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RestructureInfo RTree_restructure_tree_work(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb,
    std::tr1::unordered_set<typename CutNode::CutType*>* affected_cuts)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    RestructureInfo result;

    // The basic approach is to work bottom up, looking for "inefficient" nodes.

    // We can't do anything about root leaves directly
    if (root->leaf())
        return result;

    // Allow children to restructure first
    for(int i = 0; i < root->size(); i++) {
        RestructureInfo child_restructured = RTree_restructure_tree_work(root->node(i), loc, t, cb, affected_cuts);
        result += child_restructured;
    }
    if (result.restructures > 0) // some descendent restructured
        root->recomputeData(loc, t, cb);

    // A node is considered inefficient if there is a lot of overlap between its
    // children.  We can tell if there's a lot of overlap by comparing the sum
    // of the children's volume with the parents volume.
    float this_volume = root->data().volume();
    float children_volume = 0.f;
    for(int i = 0; i < root->size(); i++) {
        children_volume += root->node(i)->data().volume();
    }

    if (children_volume / this_volume <= 2.f)
        return result;

    if (root->node(0)->leaf())
        RTree_restructure_nodes_children<SimulationTraits, NodeData, CutNode, typename LocationServiceCache<SimulationTraits>::Iterator, typename RTreeNodeType::ObjectChildOperations>(root, loc, t, cb, affected_cuts);
    else
        RTree_restructure_nodes_children<SimulationTraits, NodeData, CutNode, RTreeNodeType*, typename RTreeNodeType::NodeChildOperations>(root, loc, t, cb, affected_cuts);

    result.restructures += 1;

    root->recomputeData(loc, t, cb);

    return result;
}

/* Recursively restructure the tree by looking for nodes with children that have
 * become inefficient and restructuring them. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RestructureInfo RTree_restructure_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    // This is just the driver which runs the process and then takes care of the
    // last step of cleaning up cuts.
    RestructureInfo result;

    typedef typename CutNode::CutType Cut;
    typedef std::tr1::unordered_set<Cut*> CutSet;
    CutSet affected_cuts;
    result = RTree_restructure_tree_work(root, loc, t, cb, &affected_cuts);

    // Finally, we take our list of cuts we compiled at the beginning and have
    // them reorganize their cut node list.  All the *cut nodes* should have
    // remained valid, we just need to get them sorted back in the right order.
    for(typename CutSet::iterator it = affected_cuts.begin(); it != affected_cuts.end(); it++) {
        Cut* cut = *it;
        cut->rebuildCutOrder();
    }

    result.cutRebuilds = affected_cuts.size();

    return result;
}


} // namespace Prox

#endif //_PROX_RTREE_RESTRUCTURE_HPP_
