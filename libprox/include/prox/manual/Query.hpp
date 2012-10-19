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

enum ManualQueryOpResult {
    // Everything worked
    MANUAL_QUERY_OP_OK,
    // The requested node was not found in the cut, probably just out of date info
    MANUAL_QUERY_OP_NODE_NOT_IN_CUT,
    // Coarsen ops can fail due to hitting a root node
    MANUAL_QUERY_OP_NODE_IS_ROOT,
    // Refine ops can fail due to having no children. Note this is different
    // from being a leaf -- it may have children in the global tree, but we
    // don't have data  replicated so we can't refine it
    MANUAL_QUERY_OP_NODE_HAS_NO_CHILDREN,
    // And we can't refine real leaf objects.
    MANUAL_QUERY_OP_NODE_IS_LEAF
};
inline const char* ManualQueryOpResultAsString(ManualQueryOpResult res) {
    switch(res) {
      case MANUAL_QUERY_OP_OK:
        return "ok";
      case MANUAL_QUERY_OP_NODE_NOT_IN_CUT:
        return "node not in cut";
      case MANUAL_QUERY_OP_NODE_IS_ROOT:
        return "node is root";
      case MANUAL_QUERY_OP_NODE_HAS_NO_CHILDREN:
        return "node has no children";
      case MANUAL_QUERY_OP_NODE_IS_LEAF:
        return "node is leaf";
      default:
        return "uknown";
    };
}

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
    ManualQueryOpResult refine(const ObjectID& objid) {
        return QueryBaseType::handler()->refine(this, objid);
    }

    /** Coarsen the query results to the given node. Return true if successful. */
    ManualQueryOpResult coarsen(const ObjectID& objid) {
        return QueryBaseType::handler()->coarsen(this, objid);
    }

    virtual uint32 numResults() const { return QueryBaseType::handler()->numResultsForQuery(this); }
    virtual uint32 size() const { return QueryBaseType::handler()->sizeForQuery(this); }

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
