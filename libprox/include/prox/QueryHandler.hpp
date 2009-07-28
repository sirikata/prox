/*  libprox
 *  QueryHandler.hpp
 *
 *  Copyright (c) 2009, Ewen Cheslack-Postava
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

#ifndef _PROX_QUERY_HANDLER_HPP_
#define _PROX_QUERY_HANDLER_HPP_

#include <prox/Query.hpp>
#include <prox/LocationServiceCache.hpp>
#include <prox/DefaultSimulationTraits.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class QueryHandler : public LocationUpdateListener<SimulationTraits>, public QueryChangeListener<SimulationTraits> {
public:
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef Query<SimulationTraits> QueryType;

    QueryHandler()
     : LocationUpdateListenerType(),
       QueryChangeListenerType()
    {}
    virtual ~QueryHandler() {}

    virtual void initialize(LocationServiceCacheType* loc_cache) = 0;
    virtual void registerQuery(QueryType* query) = 0;
    virtual void tick(const Time& t) = 0;

    // LocationUpdateListener
    virtual void locationConnected(const ObjectID& obj_id, const MotionVector3& pos, const BoundingSphere& bounds) = 0;
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void locationBoundsUpdated(const ObjectID& obj_id, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) = 0;
    virtual void locationDisconnected(const ObjectID& obj_id) = 0;

    // QueryChangeListener
    virtual void queryPositionUpdated(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;
}; // class QueryHandler

} // namespace Prox

#endif //_PROX_QUERY_HANDLER_HPP_
