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

namespace Prox {

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
 */
template<typename SimulationTraits>
class AggregateListener {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;

    AggregateListener() {}
    virtual ~AggregateListener() {}

    virtual void aggregateCreated(const ObjectIDType& objid) = 0;
    virtual void aggregateChildAdded(const ObjectIDType& objid, const ObjectIDType& child) = 0;
    virtual void aggregateChildRemoved(const ObjectIDType& objid, const ObjectIDType& child) = 0;
    virtual void aggregateDestroyed(const ObjectIDType& objid) = 0;

    virtual void aggregateObserved(const ObjectIDType& objid, uint32 nobservers) = 0;

}; // class AggregateListener

} // namespace Prox

#endif //_PROX_AGGREGATE_LISTENER_HPP_
