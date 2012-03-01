// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_GEOM_IMPL_BRUTEFORCE_NODE_ITERATOR_HPP_
#define _PROX_GEOM_IMPL_BRUTEFORCE_NODE_ITERATOR_HPP_

#include <prox/base/impl/NodeIterator.hpp>

namespace Prox {

template<typename SimulationTraits>
class BruteForceQueryHandler;

namespace BruteForceQueryHandlerImpl {

template<typename SimulationTraits>
class NodeIteratorImpl : public Prox::QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> {
public:
    typedef typename Prox::QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorBase;
    typedef typename BruteForceQueryHandler<SimulationTraits>::ObjectSetIterator ObjectSetIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;

    NodeIteratorImpl(ObjectSetIterator oit)
     : NodeIteratorBase(),
       it(oit)
    {}
    virtual ~NodeIteratorImpl() {}

    virtual NodeIteratorImpl* _clone() {
        return new NodeIteratorImpl(it);
    }

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
        return it->first;
    }
private:
    ObjectSetIterator it;
};


} // namespace BruteForceQueryHandlerImpl
} // namespace Prox

#endif //_PROX_GEOM_IMPL_BRUTEFORCE_NODE_ITERATOR_HPP_
