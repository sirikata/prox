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

#include <prox/geom/Query.hpp>
#include <prox/base/QueryHandlerBase.hpp>
#include <prox/base/DefaultSimulationTraits.hpp>
#include <prox/base/Aggregator.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class QueryHandler :
        public QueryHandlerBase< SimulationTraits, Query<SimulationTraits>, QueryChangeListener<SimulationTraits> >
{
public:
    typedef QueryHandlerBase< SimulationTraits, Query<SimulationTraits>, QueryChangeListener<SimulationTraits> > BaseType;

    typedef SimulationTraits SimulationTraitsType;

    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef Query<SimulationTraits> QueryType;
    typedef typename QueryType::ID QueryID;

    typedef typename BaseType::ShouldTrackCallback ShouldTrackCallback;
    typedef typename BaseType::ObjectList ObjectList;

    QueryHandler()
     : BaseType(),
       mQueryIDSource(0),
       mTrackChecks(false),
       mShouldRestructure(false),
       mReportRestructures(false),
       mReportHealth(false),
       mReportHealthFrequency(1),
       mReportCost(false),
       mItsSinceReportedHealth(0),
       mReportQueryStats(false)
    {}
    virtual ~QueryHandler() {}


    void trackChecks(bool t) { mTrackChecks = t; }
    void shouldRestructure(bool r) { mShouldRestructure = r; }
    void setReportRestructures(bool r) { mReportRestructures = r; }
    bool reportRestructures() const { return mReportRestructures; }
    void reportHealth(bool r) { mReportHealth = r; }
    void reportHealthFrequency(int its) { mReportHealthFrequency = its; }
    void reportCost(bool r) { mReportCost = r; }
    void reportQueryStats(bool r) { mReportQueryStats = r; }

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

    virtual void rebuild() = 0;

    /** Get an estimation of the cost of evaluating a query on this query
     *  handler.
     */
    virtual float cost() = 0;

protected:
    using BaseType::handlerID;

    virtual void registerQuery(QueryType* query) = 0;

    QueryID mQueryIDSource;

    // Whether to track constraint checks
    bool mTrackChecks;
    bool mShouldRestructure;
    bool mReportRestructures;
    bool mReportHealth;
    int mReportHealthFrequency;
    bool mReportCost;
    int mItsSinceReportedHealth;
    bool mReportQueryStats;
}; // class QueryHandler

} // namespace Prox

#endif //_PROX_QUERY_HANDLER_HPP_
