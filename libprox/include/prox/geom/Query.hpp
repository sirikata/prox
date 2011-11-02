/*  libprox
 *  Query.hpp
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

#ifndef _PROX_QUERY_HPP_
#define _PROX_QUERY_HPP_

#include <prox/util/Platform.hpp>
#include <prox/base/QueryBase.hpp>
#include <prox/base/QueryEventListener.hpp>
#include <prox/geom/QueryChangeListener.hpp>

namespace Prox {

template<typename SimulationTraits>
class QueryHandler;

template<typename SimulationTraits = DefaultSimulationTraits>
class Query :
        public QueryBase< SimulationTraits, Query<SimulationTraits>, QueryHandler<SimulationTraits>, QueryChangeListener<SimulationTraits> >
{
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::TimeType Time;

    typedef QueryBase< SimulationTraits, Query<SimulationTraits>, QueryHandler<SimulationTraits>, QueryChangeListener<SimulationTraits> > QueryBaseType;

    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef QueryEventListener< SimulationTraits, Query<SimulationTraits> > QueryEventListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef int ID;

    ~Query() {}

    /// Minimum solid angle of an object which still allows it to be returned as
    /// a result.
    const SolidAngle& angle() const {
        return mMinSolidAngle;
    }
    /// Maximum distance from the object to still return results from. This adds
    /// a distance query component to the query.
    const float radius() const {
        return mMaxRadius;
    }
    /// Maximum number of results this query should return. This adds
    /// a cap to the result set size.
    const uint32 maxResults() const {
        return mMaxResults;
    }

    void angle(const SolidAngle& new_angle) {
        assert(QueryBaseType::mValid);
        SolidAngle old_angle = mMinSolidAngle;
        mMinSolidAngle = new_angle;
        for(typename QueryBaseType::ChangeListenerListIterator it = QueryBaseType::mChangeListeners.begin(); it != QueryBaseType::mChangeListeners.end(); it++)
            (*it)->queryAngleChanged(this, old_angle, new_angle);
    }

    void maxResults(const uint32 new_mr) {
        assert(QueryBaseType::mValid);
        uint32 old_mr = mMaxResults;
        mMaxResults = new_mr;
        for(typename QueryBaseType::ChangeListenerListIterator it = QueryBaseType::mChangeListeners.begin(); it != QueryBaseType::mChangeListeners.end(); it++)
            (*it)->queryMaxResultsChanged(this, old_mr, new_mr);
    }



protected:
    friend class QueryHandler<SimulationTraits>;

    Query();

    Query(QueryHandlerType* parent, ID id, const MotionVector3& pos, const BoundingSphere& region, real maxSize, const SolidAngle& minAngle)
     : QueryBaseType(parent, id, pos, region, maxSize),
       mMinSolidAngle(minAngle),
       mMaxRadius(SimulationTraits::InfiniteRadius),
       mMaxResults(SimulationTraits::InfiniteResults)
    {
    }

    Query(QueryHandlerType* parent, ID id, const MotionVector3& pos, const BoundingSphere& region, real maxSize, const SolidAngle& minAngle, real radius)
     : QueryBaseType(parent, id, pos, region, maxSize),
       mMinSolidAngle(minAngle),
       mMaxRadius(radius),
       mMaxResults(SimulationTraits::InfiniteResults)
    {
    }

    SolidAngle mMinSolidAngle;
    real mMaxRadius;
    uint32 mMaxResults;
}; // class Query

} // namespace Prox

#endif //_PROX_QUERY_HPP_
