/*  libprox
 *  AggregateListener.hpp
 *
 *  Copyright (c) 2010, Ewen Cheslack-Postava
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of libprox nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PROX_AGGREGATE_LISTENER_HPP_
#define _PROX_AGGREGATE_LISTENER_HPP_

#include <prox/util/Platform.hpp>

namespace Prox {

template<typename SimulationTraits>
class Aggregator;

/** An AggregateListener is informed about updates to aggregates.  Aggregates
 *  are collections of objects which may be returned because they would satisfy
 *  a query, but none of their children would.
 *
 *  AggregateListeners learn about existence and membership events -- when
 *  aggregates are generated and when their membership changes -- as well as
 *  observance events -- when a query is actually aware of them.  For observance
 *  events, the listener is only notified when the number of observers changes
 *  in a significant way (zero to non-zero, or non-zero to zero) so the listener
 *  knows whether the object is in use.
 *
 *  The bounding information about the bounds so data can be generated
 *  for a location service.  Because we currently assume periodic
 *  updates of bounds, these values only include a static bounding
 *  volume, and no TimedMotionVector for movement.  Instead,
 *  aggregateBoundsUpdated will be invoked regularly if there are
 *  changes.  The trivial way to convert this to location and bounds
 *  is to use the center as the location and change the bounds center
 *  to the origin.
 *
 *  NOTE: Callbacks may happen from any thread since they may occur during
 *  normal operation as well as during rebuilding (which occurs in a separate
 *  thread). Listeners are responsible for ensuring the callbacks are
 *  thread-safe.
 */
template<typename SimulationTraits>
class AggregateListener {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;
    typedef typename SimulationTraits::realType realType;
    typedef typename SimulationTraits::Vector3Type Vector3Type;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphereType;
    typedef Aggregator<SimulationTraits> AggregatorType;

    AggregateListener() {}
    virtual ~AggregateListener() {}

    virtual void aggregateCreated(AggregatorType* handler, const ObjectIDType& objid) = 0;
    virtual void aggregateChildAdded(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) = 0;
    virtual void aggregateChildRemoved(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) = 0;
    // Only invoked on pure bounds updates. Passed as part of the
    // callback for childAdded and childRemoved.
    virtual void aggregateBoundsUpdated(AggregatorType* handler, const ObjectIDType& objid,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) = 0;
    // Notification if the query data was updated.
    virtual void aggregateQueryDataUpdated(AggregatorType* handler, const ObjectIDType& objid,
        const String& extra_query_data) = 0;
    virtual void aggregateDestroyed(AggregatorType* handler, const ObjectIDType& objid) = 0;

    // Called when the number of observers (queries with the node in their
    // results) changes. # of children below this node (objects or nodes) is
    // also provided.
    virtual void aggregateObserved(AggregatorType* handler, const ObjectIDType& objid, uint32 nobservers, uint32 nchildren) = 0;

}; // class AggregateListener

} // namespace Prox

#endif //_PROX_AGGREGATE_LISTENER_HPP_
