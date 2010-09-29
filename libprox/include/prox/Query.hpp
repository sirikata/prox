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

#include <prox/Platform.hpp>
#include <prox/DefaultSimulationTraits.hpp>
#include <prox/QueryEvent.hpp>
#include <prox/QueryEventListener.hpp>
#include <prox/QueryChangeListener.hpp>
#include <boost/thread.hpp>

namespace Prox {

template<typename SimulationTraits>
class QueryHandler;

template<typename SimulationTraits = DefaultSimulationTraits>
class Query {
public:
    typedef typename SimulationTraits::realType real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::TimeType Time;

    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef QueryEventListener<SimulationTraits> QueryEventListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef int ID;

    ~Query() {
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryDeleted(this);
    }

    QueryHandlerType* handler() const { return mParent; }

    ID id() const { return mID; }

    const MotionVector3& position() const {
        return mPosition;
    }
    Vector3 position(const Time& t) const {
        return mPosition.position(t);
    }
    const BoundingSphere& region() const {
        return mRegion;
    }
    const real maxSize() const {
        return mMaxSize;
    }
    const SolidAngle& angle() const {
        return mMinSolidAngle;
    }
    const float radius() const {
        return mMaxRadius;
    }

    void position(const MotionVector3& new_pos) {
        MotionVector3 old_pos = mPosition;
        mPosition = new_pos;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryPositionChanged(this, old_pos, new_pos);
    }

    void region(const BoundingSphere& new_region) {
        BoundingSphere old_region = mRegion;
        mRegion = new_region;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryRegionChanged(this, old_region, new_region);
    }

    void maxSize(real new_ms) {
        real old_ms = mMaxSize;
        mMaxSize = new_ms;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryMaxSizeChanged(this, old_ms, new_ms);
    }

    void angle(const SolidAngle& new_angle) {
        SolidAngle old_angle = mMinSolidAngle;
        mMinSolidAngle = new_angle;
        for(ChangeListenerListIterator it = mChangeListeners.begin(); it != mChangeListeners.end(); it++)
            (*it)->queryAngleChanged(this, old_angle, new_angle);
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
    }

    void removeEventListener() {
        mEventListener = NULL;
    }


    void pushEvent(const QueryEventType& evt) {
        {
            boost::mutex::scoped_lock lock(mEventQueueMutex);

            mEventQueue.push_back(evt);

            if (mNotified) return;
            mNotified = true;
        }

        if (mEventListener != NULL)
            mEventListener->queryHasEvents(this);
    }

    void pushEvents(std::deque<QueryEventType>& evts) {
        {
            boost::mutex::scoped_lock lock(mEventQueueMutex);

            while( !evts.empty() ) {
                mEventQueue.push_back( evts.front() );
                evts.pop_front();
            }

            if (mNotified) return;

            mNotified = true;
        }

        if (mEventListener != NULL)
            mEventListener->queryHasEvents(this);
    }

    void popEvents(std::deque<QueryEventType>& evts) {
        boost::mutex::scoped_lock lock(mEventQueueMutex);

        assert( evts.empty() );
        mEventQueue.swap(evts);
        mNotified = false;
    }

protected:
    friend class QueryHandler<SimulationTraits>;

    Query();

    Query(QueryHandlerType* parent, ID id, const MotionVector3& pos, const BoundingSphere& region, real maxSize, const SolidAngle& minAngle)
     : mParent(parent),
       mID(id),
       mPosition(pos),
       mRegion(region),
       mMaxSize(maxSize),
       mMinSolidAngle(minAngle),
       mMaxRadius(SimulationTraits::InfiniteRadius),
       mChangeListeners(),
       mEventListener(NULL),
       mNotified(false)
    {
    }

    Query(QueryHandlerType* parent, ID id, const MotionVector3& pos, const BoundingSphere& region, real maxSize, const SolidAngle& minAngle, real radius)
     : mParent(parent),
       mID(id),
       mPosition(pos),
       mRegion(region),
       mMaxSize(maxSize),
       mMinSolidAngle(minAngle),
       mMaxRadius(radius),
       mNotified(false)
    {
    }

    QueryHandlerType* mParent;
    ID mID;

    MotionVector3 mPosition;
    BoundingSphere mRegion;
    real mMaxSize;
    SolidAngle mMinSolidAngle;
    real mMaxRadius;

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

#endif //_PROX_QUERY_HPP_
