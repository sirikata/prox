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
#include <prox/base/Defs.hpp>
#include <prox/base/LocationServiceCache.hpp>

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
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;

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

    // Reparenting moves an object or node (and its entire subtree) to
    // a new node. This is like a remove/add sequence except it allows
    // subtrees to move without having to move all their elements.
    class Reparent : private Action {
    public:
        Reparent(ObjectID id, ObjectType type, ObjectID old_parent, ObjectID new_parent)
         : Action(id),
           mType(type),
           mOldParent(old_parent),
           mNewParent(new_parent)
        {}
        // Helper that fills in the ID and parent info from nodes
        // (currently based of RTreeNode).
        template <typename NodeType>
        Reparent(const NodeType* node, const NodeType* old_parent, const NodeType* new_parent)
         : Action(node->aggregateID()),
           mType(Imposter),
           mOldParent(old_parent->aggregateID()),
           mNewParent(new_parent->aggregateID())
        {}

        using Action::id;
        ObjectType type() const { return mType; }
        ObjectID oldParent() const { return mOldParent; }
        ObjectID newParent() const { return mNewParent; }
    private:
        ObjectType mType;
        ObjectID mOldParent;
        ObjectID mNewParent;
    };
    typedef std::vector<Reparent> ReparentList;

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

    // We need the location service cache so we can make sure
    // everything we use in this QueryEvent is properly maintained.
    QueryEvent(LocationServiceCacheType* loccache, QueryHandlerIndexID iid)
     : mLocCache(loccache),
       mIndexID(iid)
    {}
    QueryEvent(const QueryEvent& rhs)
     : mLocCache(rhs.mLocCache),
       mIndexID(rhs.mIndexID)
    {
        for(size_t i = 0; i < rhs.mAdditions.size(); i++)
            addAddition(rhs.mAdditions[i]);
        for(size_t i = 0; i < rhs.mReparents.size(); i++)
            addReparent(rhs.mReparents[i]);
        for(size_t i = 0; i < rhs.mRemovals.size(); i++)
            addRemoval(rhs.mRemovals[i]);
    }
    QueryEvent& operator=(const QueryEvent& rhs) {
        for(size_t i = 0; i < mAdditions.size(); i++)
            mLocCache->stopRefcountTracking(mAdditions[i].id());
        for(size_t i = 0; i < mReparents.size(); i++)
            mLocCache->stopRefcountTracking(mReparents[i].id());
        for(size_t i = 0; i < mRemovals.size(); i++)
            mLocCache->stopRefcountTracking(mRemovals[i].id());

        mLocCache = rhs.mLocCache;
        mIndexID = rhs.mIndexID;
        mAdditions.clear();
        mReparents.clear();
        mRemovals.clear();

        for(size_t i = 0; i < rhs.mAdditions.size(); i++)
            addAddition(rhs.mAdditions[i]);
        for(size_t i = 0; i < rhs.mReparents.size(); i++)
            addReparent(rhs.mReparents[i]);
        for(size_t i = 0; i < rhs.mRemovals.size(); i++)
            addRemoval(rhs.mRemovals[i]);

        return *this;
    }
    ~QueryEvent() {
        for(size_t i = 0; i < mAdditions.size(); i++)
            mLocCache->stopRefcountTracking(mAdditions[i].id());
        for(size_t i = 0; i < mReparents.size(); i++)
            mLocCache->stopRefcountTracking(mReparents[i].id());
        for(size_t i = 0; i < mRemovals.size(); i++)
            mLocCache->stopRefcountTracking(mRemovals[i].id());
    }

    const QueryHandlerIndexID indexID() const { return mIndexID; }

    void addAddition(const Addition& a) {
        mLocCache->startRefcountTracking(a.id());
        mAdditions.push_back(a);
    }
    const AdditionList& additions() const { return mAdditions; }

    void addReparent(const Reparent& a) {
        mLocCache->startRefcountTracking(a.id());
        mReparents.push_back(a);
    }
    const ReparentList& reparents() const { return mReparents; }

    void addRemoval(const Removal& r) {
        mLocCache->startRefcountTracking(r.id());
        mRemovals.push_back(r);
    }
    const RemovalList& removals() const { return mRemovals; }

    uint32 size() const { return mAdditions.size() + mReparents.size() + mRemovals.size(); }
    bool empty() const { return (mAdditions.empty() && mReparents.empty() && mRemovals.empty()); }

private:
    LocationServiceCacheType* mLocCache;
    QueryHandlerIndexID mIndexID;
    AdditionList mAdditions;
    ReparentList mReparents;
    RemovalList mRemovals;
}; // class QueryEvent

} // namespace Prox

#endif //_PROX_QUERY_EVENT_LISTENER_HPP_
