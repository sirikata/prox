// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_GEOM_IMPL_LEVEL_NODE_ITERATOR_HPP_
#define _PROX_GEOM_IMPL_LEVEL_NODE_ITERATOR_HPP_

#include <prox/rtree/impl/NodeIterator.hpp>

namespace Prox {

template<typename SimulationTraits>
class LevelQueryHandler;

namespace LevelQueryHandlerImpl {

template<typename SimulationTraits>
class NodeIteratorImpl :
        public RTreeHandlerImpl::NodeIteratorImpl<SimulationTraits, typename LevelQueryHandler<SimulationTraits>::RTree>
{
public:
    typedef typename RTreeHandlerImpl::NodeIteratorImpl<SimulationTraits, typename LevelQueryHandler<SimulationTraits>::RTree> Base;

    NodeIteratorImpl(typename Base::RTreeNodeIterator rit)
     : Base(rit)
    {}
    virtual ~NodeIteratorImpl() {}

    virtual NodeIteratorImpl* _clone() {
        return new NodeIteratorImpl(Base::it);
    }
};


} // namespace LevelQueryHandlerImpl
} // namespace Prox

#endif //_PROX_GEOM_IMPL_LEVEL_NODE_ITERATOR_HPP_
