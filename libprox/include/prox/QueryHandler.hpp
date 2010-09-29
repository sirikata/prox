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
#include <prox/AggregateListener.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class QueryHandler : public LocationUpdateListener<SimulationTraits>, public QueryChangeListener<SimulationTraits> {
public:
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;
    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef Query<SimulationTraits> QueryType;
    typedef typename QueryType::ID QueryID;

    QueryHandler()
     : LocationUpdateListenerType(),
       QueryChangeListenerType(),
       mQueryIDSource(0),
       mAggregateListener(NULL),
       mTrackChecks(false),
       mShouldRestructure(false)
    {}
    virtual ~QueryHandler() {}


    void trackChecks(bool t) { mTrackChecks = t; }
    void shouldRestructure(bool r) { mShouldRestructure = r; }

    virtual void initialize(LocationServiceCacheType* loc_cache) = 0;

    QueryType* registerQuery(const MotionVector3& pos, const BoundingSphere& region, Real maxSize, const SolidAngle& minAngle) {
        QueryType* q = new QueryType(this, mQueryIDSource++, pos, region, maxSize, minAngle);
        registerQuery(q);
        return q;
    }
    QueryType* registerQuery(const MotionVector3& pos, const BoundingSphere& region, Real maxSize, const SolidAngle& minAngle, float radius) {
        QueryType* q = new QueryType(this, mQueryIDSource++, pos, region, maxSize, minAngle, radius);
        registerQuery(q);
        return q;
    }

    virtual void tick(const Time& t) = 0;

    virtual uint32 numObjects() const = 0;
    virtual uint32 numQueries() const = 0;

    void setAggregateListener(AggregateListenerType* listener) {
        mAggregateListener = listener;
    }
    void removeAggregateListener() {
        mAggregateListener = NULL;
    }

    virtual LocationServiceCacheType* locationCache() const = 0;

    // LocationUpdateListener
    virtual void locationConnected(const ObjectID& obj_id, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) = 0;
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) = 0;
    virtual void locationDisconnected(const ObjectID& obj_id) = 0;

    // QueryChangeListener
    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void queryMaxSizeChanged(QueryType* query, Real old_ms, Real new_ms) = 0;
    virtual void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;

protected:
    virtual void registerQuery(QueryType* query) = 0;

    QueryID mQueryIDSource;
    AggregateListenerType* mAggregateListener;

    // Whether to track constraint checks
    bool mTrackChecks;
    bool mShouldRestructure;
}; // class QueryHandler

} // namespace Prox

#endif //_PROX_QUERY_HANDLER_HPP_
