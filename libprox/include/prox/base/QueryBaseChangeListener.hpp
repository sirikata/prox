// Copyright (c) 2011 libprox Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_QUERY_BASE_CHANGE_LISTENER_HPP_
#define _PROX_QUERY_BASE_CHANGE_LISTENER_HPP_

namespace Prox {

// Base class for listeners to queries. Only includes core, required events, and
// can support different types of query classes. Essentially, this only includes
// events about a) the existence of the query and b) properties of the querier
// itself (position, coverage). It *doesn't* include query-specific information
// like minimum solid angle or maximum number of results.
template<typename SimulationTraits, typename QueryTypeT>
class QueryBaseChangeListener {
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef QueryTypeT QueryType;

    QueryBaseChangeListener() {}
    virtual ~QueryBaseChangeListener() {}

    /** The center position of the querier changed. */
    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    /** The region covered by the *centers* of queriers has changed. NOTE that
     *  this isn't the region of the query -- that must also include the maximum
     *  object size. For a single object querier this should be a degenerate
     *  (zero-volume) region.
     */
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    /** The maximum size of the the objects that make up this query has changed.
     *  (For a single-object querier this is just the size of the object.)
     */
    virtual void queryMaxSizeChanged(QueryType* query, real old_ms, real new_ms) = 0;
    // If implicit is true, then the query was destroyed as part of the deletion
    // process and nobody will be able to collect any more results from the query.
    virtual void queryDestroyed(QueryType* query, bool implicit) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;

}; // class QueryChangeListener

} // namespace Prox

#endif //_PROX_QUERY_BASE_CHANGE_LISTENER_HPP_
