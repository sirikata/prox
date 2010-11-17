/*  libprox
 *  RTreeCost.hpp
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

#include <iostream>

#ifndef _PROX_RTREE_COST_HPP_
#define _PROX_RTREE_COST_HPP_

#define COST_NODE_TEST 1.f
#define COST_LEAF_TEST 1.f

namespace Prox {

/** Compute (an estimate of) the cost of traversing the tree rooted a the given
 *  node. This does not include the cost of checking the root node; see
 *  RTree_cost for a version that does that.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
float RTree_traverse_cost(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t
) {

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // Our cost function is straightforward: the cost at each node is the cost
    // of testing the node plus the conditional cost of traversing the
    // children. The conditional cost is simply the cost of the subtree times
    // the probability of traversing the subtree.  In other words, the cost is
    // defined recursively as:
    //
    // C(n) = \sum_(n_i in children) (C_check + p(n_i | n) * C(n_i))
    //
    // where p(n_i | n) is the probability of traversing n_i given that its
    // parent, n, is traversed

    float cost = 0.f;

    NodeData this_data = node->data();
    for(typename RTreeNodeType::Index i = 0; i < node->size(); i++) {
        NodeData child_data = node->childData(i, loc, t);
        float p = NodeData::hitProbability(this_data, child_data);
        float child_cost = (node->leaf() ? COST_LEAF_TEST : RTree_traverse_cost(node->node(i), loc, t));
        cost += COST_NODE_TEST + p * child_cost;
    }

    return cost;
}

/** Compute (an estimate of) the cost of traversing the tree rooted a the given
 *  node. This does not include the cost of checking the root node; see
 *  RTree_cost for a version that does that.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
float RTree_cost(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t
) {
    return COST_NODE_TEST + RTree_traverse_cost(node, loc, t);
}

} // namespace Prox

#endif //_PROX_RTREE_COST_HPP_
