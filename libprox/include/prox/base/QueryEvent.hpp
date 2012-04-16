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

#include <prox/base/DefaultSimulationTraits.hpp>

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
    typedef typename SimulationTraits::ObjectIDNullType ObjectIDNull;

    enum ObjectType {
        Normal,
        Imposter
    };
    /** The permanence of a change. The meaning of this depends on
     *  whether you're querying for a cut or for a subtree.
     *
     *  Querying for a cut:
     *   * Transient - the cut moved around so that the object isn't
     *       in the cut anymore, but the object is still in the
     *       tree. It could reappear in the cut soon.
     *   * Permanent - the object was actually deleted from the tree.
     *
     *  Querying for a subtree for replication:
     *   * Transient - the cut moved around so that the object isn't
     *       in the cut anymore, but the object is still in the
     *       tree. It could reappear in the cut soon.
     *   * Permanent - the object was actually deleted from the tree
     *       or the cut moved up and the object/node doesn't need to
     *       be maintained in the replicated tree anymore. This works
     *       because replication performed so an additional layer of
     *       querying can be performed. That layer can worry about
     *       transient removals for individual queries.
     */
    enum ObjectEventPermanence {
        Transient,
        Permanent
    };

    class Action {
    public:
        Action(ObjectID id)
         : mID(id)
        {}

        ObjectID id() const { return mID; }
    private:
        Action();
        ObjectID mID;
    };
    class Addition : private Action {
    public:
        // If parent == ObjectIDNull()() this is equivalent to no parent (or
        // parent's not exposed)
        Addition(ObjectID id, ObjectType type, ObjectID parent)
         : Action(id),
           mType(type),
           mParent(parent)
        {}
        // Helper that fills in the ID and parent from some node (currently
        // based of RTreeNode). Should have aggregateID() and
        // parentAggregateID() methods.
        template <typename NodeType>
        Addition(const NodeType* node, ObjectType type)
         : Action(node->aggregateID()),
           mType(type),
           mParent(node->parentAggregateID())
        {}

        using Action::id;
        ObjectType type() const { return mType; }
        ObjectID parent() const { return mParent; }
    private:
        ObjectType mType;
        ObjectID mParent;
    };
    typedef std::vector<Addition> AdditionList;

    class Removal : private Action {
    public:
        Removal(ObjectID id, ObjectEventPermanence perm)
         : Action(id),
           mPerm(perm)
        {}

        using Action::id;
        ObjectEventPermanence permanent() const { return mPerm; }
    private:
        ObjectEventPermanence mPerm;
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
