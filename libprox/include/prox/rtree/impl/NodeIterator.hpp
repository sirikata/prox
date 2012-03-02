// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_RTREE_IMPL_RTREE_NODE_ITERATOR_HPP_
#define _PROX_RTREE_IMPL_RTREE_NODE_ITERATOR_HPP_

#include <prox/base/impl/NodeIterator.hpp>

namespace Prox {

namespace RTreeHandlerImpl {

/** All the iterators for RTree-based classes are essentially identical except
 *  for the RTree typedef. This implements the basic interface with a template
 *  parameter specifying the RTree type so we can use a simple subclass that
 *  overrides just some construction methods to get the full functionality.
 */
template<typename SimulationTraits, typename RTreeType>
class NodeIteratorImpl : public Prox::QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> {
public:
    typedef typename Prox::QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorBase;
    typedef RTreeType RTree;
    typedef typename RTree::NodeIterator RTreeNodeIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::TimeType Time;

    NodeIteratorImpl(RTreeNodeIterator rit)
     : NodeIteratorBase(),
       it(rit)
    {}
    virtual ~NodeIteratorImpl() {}

    // Traversal
    virtual void next() {
        it++;
    }

    // Comparison
    virtual bool _equals(NodeIteratorBase* rhs_base) {
        NodeIteratorImpl* rhs = dynamic_cast<NodeIteratorImpl*>(rhs_base);
        if (rhs == NULL) return false;
        return (it == rhs->it);
    }

    // Get data
    const ObjectID& id() const {
        return it.id();
    }
    ObjectID parentId() const {
        return it.parentId();
    }
    BoundingSphere bounds(const Time& t) const {
        return it.bounds(t);
    }
protected:
    RTreeNodeIterator it;
};


} // namespace RTreeHandlerImpl
} // namespace Prox

#endif //_PROX_RTREE_IMPL_RTREE_NODE_ITERATOR_HPP_
