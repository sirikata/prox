// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_MANUAL_QUERY_HANDLER_HPP_
#define _PROX_MANUAL_QUERY_HANDLER_HPP_

#include <prox/manual/Query.hpp>
#include <prox/base/QueryHandlerBase.hpp>
#include <prox/base/DefaultSimulationTraits.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class ManualQueryHandler :
        public QueryHandlerBase< SimulationTraits, ManualQuery<SimulationTraits>, ManualQueryChangeListener<SimulationTraits> >
{
public:
    typedef QueryHandlerBase< SimulationTraits, ManualQuery<SimulationTraits>, ManualQueryChangeListener<SimulationTraits> > BaseType;

    typedef SimulationTraits SimulationTraitsType;

    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef ManualQueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef ManualQuery<SimulationTraits> QueryType;
    typedef typename QueryType::ID QueryID;

    typedef typename BaseType::ShouldTrackCallback ShouldTrackCallback;
    typedef typename BaseType::ObjectList ObjectList;
    typedef typename BaseType::NodeIterator NodeIterator;
    typedef typename BaseType::NodeIteratorImpl NodeIteratorImpl;

    ManualQueryHandler()
     : BaseType()
    {}
    virtual ~ManualQueryHandler() {}

    QueryType* registerQuery(const MotionVector3& pos, const BoundingSphere& region, Real maxSize) {
        QueryType* q = new QueryType(this, mQueryIDSource++, pos, region, maxSize);
        registerQuery(q);
        return q;
    }

protected:
    // Friend ManualQueries so they can forward requests to us
    friend class ManualQuery<SimulationTraits>;

    using BaseType::handlerID;

    virtual void registerQuery(QueryType* query) = 0;

    /** Refine a query's results by expanding the query's result cut at the
     *  specified object identifier to its children.
     */
    virtual bool refine(QueryType* query, const ObjectID& objid) = 0;
    /** Coarsen a query's results by reducing the query's result cut at the
     *  specified object identifier to its parent. This will also, obviously
     *  remove any other children (or deeper descendants) that fall under
     *  objid's parent.
     */
    virtual bool coarsen(QueryType* query, const ObjectID& objid) = 0;

    // Implementation of iterators
    virtual NodeIteratorImpl* nodesBeginImpl() const = 0;
    virtual NodeIteratorImpl* nodesEndImpl() const = 0;

    QueryID mQueryIDSource;
}; // class QueryHandler

} // namespace Prox

#endif //_PROX_MANUAL_QUERY_HANDLER_HPP_
