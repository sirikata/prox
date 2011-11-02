// Copyright (c) 2011 libprox Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_AGGREGATOR_HPP_
#define _PROX_AGGREGATOR_HPP_

namespace Prox {

template<typename SimulationTraits>
class AggregateListener;

/** Base class for systems that generate aggregates that can be exposed as their
 *  own objects. Tracks one AggregateListener.
 */
template<typename SimulationTraits>
class Aggregator {
public:
    typedef Aggregator<SimulationTraits> AggregatorType;
    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    Aggregator()
     : mAggregateListener(NULL)
    {}
    ~Aggregator() {}

    void setAggregateListener(AggregateListenerType* listener) {
        mAggregateListener = listener;
    }
    void removeAggregateListener() {
        mAggregateListener = NULL;
    }
protected:
    AggregateListenerType* mAggregateListener;
};

} // namespace Prox

#endif //_PROX_AGGREGATOR_HPP_
