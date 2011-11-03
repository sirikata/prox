// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_MANUAL_QUERY_CHANGE_LISTENER_HPP_
#define _PROX_MANUAL_QUERY_CHANGE_LISTENER_HPP_

#include <prox/base/QueryBaseChangeListener.hpp>

namespace Prox {

template<typename SimulationTraits>
class ManualQuery;

template<typename SimulationTraits>
class ManualQueryChangeListener :
        public QueryBaseChangeListener< SimulationTraits, ManualQuery<SimulationTraits> >
{
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef ManualQuery<SimulationTraits> QueryType;

    ManualQueryChangeListener() {}
    virtual ~ManualQueryChangeListener() {}

    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void queryMaxSizeChanged(QueryType* query, real old_ms, real new_ms) = 0;
    virtual void queryDestroyed(QueryType* query, bool implicit) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;
}; // class ManualQueryChangeListener

} // namespace Prox

#endif //_PROX_MANUAL_QUERY_CHANGE_LISTENER_HPP_
