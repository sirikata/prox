// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_MANUAL_QUERY_HPP_
#define _PROX_MANUAL_QUERY_HPP_

#include <prox/util/Platform.hpp>
#include <prox/base/QueryBase.hpp>
#include <prox/base/QueryEventListener.hpp>
#include <prox/manual/QueryChangeListener.hpp>

namespace Prox {

template<typename SimulationTraits>
class ManualQueryHandler;

/** ManualQueries are controlled by the querier. They assume a tree structure to
 *  objects and aggregates. All queriers start at the root and request that
 *  their results are refined at or coarsened to given nodes.
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class ManualQuery :
        public QueryBase< SimulationTraits, ManualQuery<SimulationTraits>, ManualQueryHandler<SimulationTraits>, ManualQueryChangeListener<SimulationTraits> >
{
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::ObjectIDType ObjectID;

    typedef QueryBase< SimulationTraits, ManualQuery<SimulationTraits>, ManualQueryHandler<SimulationTraits>, ManualQueryChangeListener<SimulationTraits> > QueryBaseType;
    typedef ManualQuery<SimulationTraits> QueryType;

    typedef ManualQueryHandler<SimulationTraits> QueryHandlerType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef QueryEventListener< SimulationTraits, QueryType > QueryEventListenerType;
    typedef ManualQueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef int ID;

    virtual ~ManualQuery() {}

    /** Refine the query results at the given node. Returns true if successful. */
    bool refine(const ObjectID& objid) {
        return QueryBaseType::handler()->refine(this, objid);
    }

    /** Coarsen the query results to the given node. Return true if successful. */
    bool coarsen(const ObjectID& objid) {
        return QueryBaseType::handler()->coarsen(this, objid);
    }

protected:
    friend class ManualQueryHandler<SimulationTraits>;

    ManualQuery();

    ManualQuery(QueryHandlerType* parent, ID id, const MotionVector3& pos, const BoundingSphere& region, real maxSize)
     : QueryBaseType(parent, id, pos, region, maxSize)
    {
    }

}; // class Query

} // namespace Prox

#endif //_PROX_MANUAL_QUERY_HPP_
