/*  libprox
 *  QueryChangeListener.hpp
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

#ifndef _PROX_QUERY_CHANGE_LISTENER_HPP_
#define _PROX_QUERY_CHANGE_LISTENER_HPP_

#include <prox/base/QueryBaseChangeListener.hpp>

namespace Prox {

template<typename SimulationTraits>
class Query;

template<typename SimulationTraits>
class QueryChangeListener :
        public QueryBaseChangeListener< SimulationTraits, Query<SimulationTraits> >
{
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef Query<SimulationTraits> QueryType;

    QueryChangeListener() {}
    virtual ~QueryChangeListener() {}

    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void queryMaxSizeChanged(QueryType* query, real old_ms, real new_ms) = 0;
    virtual void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) = 0;
    virtual void queryMaxResultsChanged(QueryType* query, const uint32 old_val, const uint32 new_val) = 0;
    // If implicit is true, then the query was destroyed as part of the deletion
    // process and nobody will be able to collect any more results from the query.
    virtual void queryDestroyed(QueryType* query, bool implicit) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;

}; // class QueryChangeListener

} // namespace Prox

#endif //_PROX_QUERY_CHANGE_LISTENER_HPP_
