// Copyright (c) 2011 libprox Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_QUERY_BASE_HPP_
#define _PROX_QUERY_BASE_HPP_

#include <prox/util/Platform.hpp>
#include <prox/base/DefaultSimulationTraits.hpp>
#include <prox/base/QueryEvent.hpp>
#include <prox/base/QueryEventListener.hpp>
#include <boost/thread.hpp>

namespace Prox {

template<typename SimulationTraits, typename QueryTypeT, typename QueryHandlerTypeT, typename QueryChangeListenerTypeT>
class QueryBase {
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::TimeType Time;

    typedef QueryTypeT QueryType;
    typedef QueryHandlerTypeT QueryHandlerType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef QueryEventListener<SimulationTraits, QueryTypeT > QueryEventListenerType;
    typedef QueryChangeListenerTypeT QueryChangeListenerType;

    typedef int ID;

    ~QueryBase() {
        if (mValid)
            destroy(true);

        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryDeleted((QueryType*)this);
    }

    /** Clear this query and disable the ability to change any of its
     *  values. Any remaining query results are added before the method returns
     *  and no further notifications about query event availability will be
     *  made.  If implicit == true, then the objects still in the result set do
     *  not have events created to remove them -- i.e. this method returns
     *  without changing the result set at all. This mode is used by the
     *  destructor since results cannot be collected after it exits, but can
     *  also be used if you do not need those updates but still need the query
     *  data (e.g. position, query angle, etc).
     */
    void destroy(bool implicit = false) {
        assert(mValid);
        mValid = false;

        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryDestroyed((QueryType*)this, implicit);
    }

    QueryHandlerType* handler() const { return mParent; }

    ID id() const { return mID; }

    /// Center position of the querier or aggregate querier.
    const MotionVector3& position() const {
        return mPosition;
    }
    Vector3 position(const Time& t) const {
        return mPosition.position(t);
    }
    /// Region 'covered' by the queriers, not including their extents. In other
    /// words, the bounding region of the centers of the queriers. For a single
    /// querier this will be a single .
    const BoundingSphere& region() const {
        return mRegion;
    }
    /// Maximum size of queriers. For individual queriers, the size of the
    /// querier. For aggregate queriers, the size of the largest querier.
    const real maxSize() const {
        return mMaxSize;
    }

    void position(const MotionVector3& new_pos) {
        assert(mValid);
        MotionVector3 old_pos = mPosition;
        mPosition = new_pos;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryPositionChanged((QueryType*)this, old_pos, new_pos);
    }

    void region(const BoundingSphere& new_region) {
        assert(mValid);
        BoundingSphere old_region = mRegion;
        mRegion = new_region;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryRegionChanged((QueryType*)this, old_region, new_region);
    }

    void maxSize(real new_ms) {
        assert(mValid);
        real old_ms = mMaxSize;
        mMaxSize = new_ms;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryMaxSizeChanged((QueryType*)this, old_ms, new_ms);
    }


    void addChangeListener(QueryChangeListenerType* listener) {
        mChangeListeners.push_back(listener);
    }

    void removeChangeListener(QueryChangeListenerType* listener) {
        ChangeListenerListIterator it = std::find(mChangeListeners.begin(), mChangeListeners.end(), listener);
        if (it != mChangeListeners.end())
            mChangeListeners.erase(it);
    }

    void setEventListener(QueryEventListenerType* listener) {
        mEventListener = listener;

        // Notify if we have outstanding events. This ensures events get out,
        // even if they occurred before the event listener was set, e.g. during
        // construction. No locking as this should only be happening from the
        // main thread
        if (mEventListener != NULL && mValid && !mEventQueue.empty()) {
            mNotified = true;
            mEventListener->queryHasEvents((QueryType*)this);
        }
    }

    void removeEventListener() {
        mEventListener = NULL;
    }


    void pushEvent(const QueryEventType& evt) {
        {
            boost::mutex::scoped_lock lock(mEventQueueMutex);

            mEventQueue.push_back(evt);

            if (mNotified) return;
            // Can't notify if we don't have a listener or we've gone
            // invalid. This check *must* be before marking mNotified or we can
            // get locked into always thinking we've notified when we actually
            // haven't.
            if (mEventListener == NULL || !mValid) return;

            mNotified = true;
        }

        if (mEventListener != NULL && mValid)
            mEventListener->queryHasEvents((QueryType*)this);
    }

    void pushEvents(std::deque<QueryEventType>& evts) {
        {
            boost::mutex::scoped_lock lock(mEventQueueMutex);

            while( !evts.empty() ) {
                mEventQueue.push_back( evts.front() );
                evts.pop_front();
            }

            if (mEventQueue.empty()) return;
            if (mNotified) return;
            // Can't notify if we don't have a listener or we've gone
            // invalid. This check *must* be before marking mNotified or we can
            // get locked into always thinking we've notified when we actually
            // haven't.
            if (mEventListener == NULL || !mValid) return;

            mNotified = true;
        }

        if (mEventListener != NULL && mValid)
            mEventListener->queryHasEvents((QueryType*)this);
    }

    void popEvents(std::deque<QueryEventType>& evts) {
        boost::mutex::scoped_lock lock(mEventQueueMutex);

        assert( evts.empty() );
        mEventQueue.swap(evts);
        mNotified = false;
    }

protected:
    QueryBase();

    QueryBase(QueryHandlerType* parent, ID id, const MotionVector3& pos, const BoundingSphere& region, real maxSize)
     : mParent(parent),
       mID(id),
       mPosition(pos),
       mRegion(region),
       mMaxSize(maxSize),
       mValid(true),
       mChangeListeners(),
       mEventListener(NULL),
       mNotified(false)
    {
    }

    QueryHandlerType* mParent;
    ID mID;

    MotionVector3 mPosition;
    BoundingSphere mRegion;
    real mMaxSize;

    // Whether this query is still valid. The query may be invalid (removed from
    // the query handler) but still hold remaining result events.
    bool mValid;

    typedef std::list<QueryChangeListenerType*> ChangeListenerList;
    typedef typename ChangeListenerList::iterator ChangeListenerListIterator;
    ChangeListenerList mChangeListeners;
    QueryEventListenerType* mEventListener;

    typedef std::deque<QueryEventType> EventQueue;
    EventQueue mEventQueue;
    bool mNotified; // whether we've notified event listeners of new events
    boost::mutex mEventQueueMutex;
}; // class Query

} // namespace Prox

#endif //_PROX_QUERY_BASE_HPP_
