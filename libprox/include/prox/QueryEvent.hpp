/*  libprox
 *  QueryEvent.hpp
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

#ifndef _PROX_QUERY_EVENT_HPP_
#define _PROX_QUERY_EVENT_HPP_

#include <prox/DefaultSimulationTraits.hpp>

namespace Prox {

/** Results from a query are a set of objects and imposter objects. Imposter
 *  objects represent groups of real objects, and a result set should not
 *  include imposters that are parents of objects which are in the result set.
 *
 *  A QueryEvent represents an update to the result set.  For initial results it
 *  might just be the addition of a single object or imposter objects.  For
 *  later events, it will be a refinement (add child nodes, remove parent nodes,
 *  for example, removing an imposter and adding individual objects it
 *  represented) or a reduction (remove child nodes, add parent nodes, for
 *  example replacing individual objects with an imposter).
 *
 *  QueryEvents should, in some sense, be minimal: it should not be possible to
 *  break the QueryEvent into multiple smaller QueryEvents without using
 *  intermediate nodes that weren't in the original QueryEvent. (For instance,
 *  if A has 4 grandchildren B, C, D, and E, a QueryEvent replacing A with its
 *  grandchildren is "minimal" because the only way to break it into multiple
 *  QueryEvents is to use its direct children for intermediate steps.) This
 *  allows a client to infer what objects must be processed and added before
 *  others are removed as precisely as possible.
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class QueryEvent {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;

    enum ObjectType {
        Normal,
        Imposter
    };
    /** The permanence of a change. Currently only indicates whether a
     * removal is permanent (e.g. a disconnect) or temporary (e.g. tree
     * rearranging, fell out of the result set naturally, etc).
     */
    enum ObjectEventPermanence {
        Transient,
        Permanent
    };

    class Action {
    public:
        Action(ObjectID id, ObjectType type, ObjectEventPermanence perm)
         : mID(id), mType(type), mPerm(perm)
        {}

        ObjectID id() const { return mID; }
        ObjectType type() const { return mType; }
        ObjectEventPermanence permanent() const { return mPerm; }
    private:
        Action();
        ObjectID mID;
        ObjectType mType;
        ObjectEventPermanence mPerm;
    };
    class Addition : private Action {
    public:
        Addition(ObjectID id, ObjectType type)
         : Action(id, type, Transient)
        {}

        using Action::id;
        using Action::type;
        using Action::permanent;
    };
    typedef std::vector<Addition> AdditionList;

    class Removal : private Action {
    public:
        Removal(ObjectID id, ObjectType type, ObjectEventPermanence perm)
         : Action(id, type, perm)
        {}

        using Action::id;
        using Action::type;
        using Action::permanent;
    };
    typedef std::vector<Removal> RemovalList;

    QueryEvent() {}

    AdditionList& additions() { return mAdditions; }
    const AdditionList& additions() const { return mAdditions; }

    RemovalList& removals() { return mRemovals; }
    const RemovalList& removals() const { return mRemovals; }

    uint32 size() const { return mAdditions.size() + mRemovals.size(); }
    bool empty() const { return (mAdditions.empty() && mRemovals.empty()); }

private:
    AdditionList mAdditions;
    RemovalList mRemovals;
}; // class QueryEventListener

} // namespace Prox

#endif //_PROX_QUERY_EVENT_LISTENER_HPP_
