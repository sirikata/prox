// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_GEOM_IMPL_RTREECUT_NODE_ITERATOR_HPP_
#define _PROX_GEOM_IMPL_RTREECUT_NODE_ITERATOR_HPP_

#include <prox/rtree/impl/NodeIterator.hpp>

namespace Prox {

template<typename SimulationTraits, typename NodeDataType>
class RTreeCutQueryHandler;

namespace RTreeCutQueryHandlerImpl {

template<typename SimulationTraits, typename NodeDataType>
class NodeIteratorImpl :
        public RTreeHandlerImpl::NodeIteratorImpl<SimulationTraits, typename RTreeCutQueryHandler<SimulationTraits, NodeDataType>::RTree>
{
public:
    typedef typename RTreeHandlerImpl::NodeIteratorImpl<SimulationTraits, typename RTreeCutQueryHandler<SimulationTraits, NodeDataType>::RTree> Base;

    NodeIteratorImpl(typename Base::RTreeNodeIterator rit)
     : Base(rit)
    {}
    virtual ~NodeIteratorImpl() {}

    virtual NodeIteratorImpl* _clone() {
        return new NodeIteratorImpl(Base::it);
    }
};


} // namespace RTreeCutQueryHandlerImpl
} // namespace Prox

#endif //_PROX_GEOM_IMPL_RTREECUT_NODE_ITERATOR_HPP_
