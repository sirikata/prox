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
    typedef typename Prox::BruteForceQueryHandler<SimulationTraits> QueryHandler;
    typedef typename QueryHandler::ObjectSetConstIterator ObjectSetConstIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::TimeType Time;

    NodeIteratorImpl(const QueryHandler* p, ObjectSetConstIterator oit)
     : NodeIteratorBase(),
       parent(p),
       it(oit)
    {}
    virtual ~NodeIteratorImpl() {}

    virtual NodeIteratorImpl* _clone() {
        return new NodeIteratorImpl(parent, it);
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
    ObjectID parentId() const {
        return typename SimulationTraits::ObjectIDNullType()();
    }
    BoundingSphere bounds(const Time& t) const {
        return parent->mLocCache->worldCompleteBounds(it->second, t);
    }
    uint32 cuts() const {
        return 0;
    }
private:
    const QueryHandler* parent;
    ObjectSetConstIterator it;
};


} // namespace BruteForceQueryHandlerImpl
} // namespace Prox

#endif //_PROX_GEOM_IMPL_BRUTEFORCE_NODE_ITERATOR_HPP_
