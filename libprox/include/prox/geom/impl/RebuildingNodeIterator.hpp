// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_GEOM_IMPL_REBUILDING_NODE_ITERATOR_HPP_
#define _PROX_GEOM_IMPL_REBUILDING_NODE_ITERATOR_HPP_

#include <prox/base/impl/NodeIterator.hpp>

namespace Prox {

template<typename SimulationTraits>
class RebuildingQueryHandler;

namespace RebuildingQueryHandlerImpl {

template<typename SimulationTraits>
class NodeIteratorImpl : public Prox::QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> {
private:
    // We need an extra flag to know which of the ranges we're covering because
    // the end() iterators could be identical.
    enum Phase {
        PHASE_PRIMARY,
        PHASE_REBUILDING,
        PHASE_DONE
    };
public:
    typedef typename Prox::QueryHandlerBaseImpl::NodeIterator<SimulationTraits> PublicNodeIterator;
    typedef typename Prox::QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorBase;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::TimeType Time;

    // Regular constructor. This is a meta-iterator that covers (possibly) two
    // ranges. It just tracks those two ranges and a current position in them.
    NodeIteratorImpl(
        PublicNodeIterator pb, PublicNodeIterator pe,
        PublicNodeIterator rb, PublicNodeIterator re)
     : mPrimaryBegin(pb),
       mPrimaryEnd(pe),
       mRebuildingBegin(rb),
       mRebuildingEnd(re),
       mCurrent(mPrimaryBegin),
       mPhase(PHASE_PRIMARY)
    {}
    // end() constructor. Takes same parameters as begin() constructor, plus a
    // tag indicating that this should set the current position at the end
    NodeIteratorImpl(
        PublicNodeIterator pb, PublicNodeIterator pe,
        PublicNodeIterator rb, PublicNodeIterator re,
        bool end)
     : mPrimaryBegin(pb),
       mPrimaryEnd(pe),
       mRebuildingBegin(rb),
       mRebuildingEnd(re),
       mCurrent(mRebuildingEnd),
       mPhase(PHASE_DONE)
    {}
    // clone() constructor
    NodeIteratorImpl(
        PublicNodeIterator pb, PublicNodeIterator pe,
        PublicNodeIterator rb, PublicNodeIterator re,
        PublicNodeIterator c, Phase p)
     : mPrimaryBegin(pb),
       mPrimaryEnd(pe),
       mRebuildingBegin(rb),
       mRebuildingEnd(re),
       mCurrent(c),
       mPhase(p)
    {}
    virtual ~NodeIteratorImpl() {}

    virtual NodeIteratorImpl* _clone() {
        return new NodeIteratorImpl(
            mPrimaryBegin, mPrimaryEnd,
            mRebuildingBegin, mRebuildingEnd,
            mCurrent, mPhase
        );
    }

    // Traversal
    virtual void next() {
        assert(mPhase != PHASE_DONE);

        mCurrent++;
        // Check whether we need to move into new phases. The ordering allows
        // stages that require no work to be processed immediately, allowing,
        // e.g., to go from the primary's last element direction to PHASE_DONE
        // if no rebuilding nodes exist.
        if (mPhase == PHASE_PRIMARY && mCurrent == mPrimaryEnd) {
            mCurrent = mRebuildingBegin;
            mPhase = PHASE_REBUILDING;
        }
        if (mPhase == PHASE_REBUILDING && mCurrent == mRebuildingEnd) {
            mCurrent = mRebuildingEnd;
            mPhase = PHASE_DONE;
        }
    }

    // Comparison
    virtual bool _equals(NodeIteratorBase* rhs_base) {
        NodeIteratorImpl* rhs = dynamic_cast<NodeIteratorImpl*>(rhs_base);
        if (rhs == NULL) return false;

        return (
            (mPhase == rhs->mPhase) &&
            (mCurrent == rhs->mCurrent) &&
            (mPrimaryBegin == rhs->mPrimaryBegin) &&
            (mPrimaryEnd == rhs->mPrimaryEnd) &&
            (mRebuildingBegin == rhs->mRebuildingBegin) &&
            (mRebuildingEnd == rhs->mRebuildingEnd)
        );
    }

    // Get data
    const ObjectID& id() const {
        return mCurrent.id();
    }
    ObjectID parentId() const {
        return mCurrent.parentId();
    }
    BoundingSphere bounds(const Time& t) const {
        return mCurrent.bounds(t);
    }
private:
    PublicNodeIterator mPrimaryBegin;
    PublicNodeIterator mPrimaryEnd;
    PublicNodeIterator mRebuildingBegin;
    PublicNodeIterator mRebuildingEnd;
    PublicNodeIterator mCurrent;
    Phase mPhase;
};


} // namespace RebuildingQueryHandlerImpl
} // namespace Prox

#endif //_PROX_GEOM_IMPL_REBUILDING_NODE_ITERATOR_HPP_
