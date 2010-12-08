/*  libprox
 *  RebuildingQueryHandler.hpp
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

#ifndef _PROX_REBUILDING_QUERY_HANDLER_HPP_
#define _PROX_REBUILDING_QUERY_HANDLER_HPP_

#include <prox/Query.hpp>
#include <prox/LocationServiceCache.hpp>
#include <prox/DefaultSimulationTraits.hpp>
#include <prox/AggregateListener.hpp>

namespace Prox {

/** RebuildingQueryHandler is a relatively small wrapper around a regular
 *  QueryHandler which manages the rebuilding process using two separate
 *  QueryHandlers. The rebuilding process is asynchronous and allows processing
 *  to continue, including addition and removal of objects and queries, while
 *  the rebuilding is in progress.
 */
template<typename ImplQueryHandler>
class RebuildingQueryHandler :
        public QueryHandler<typename ImplQueryHandler::SimulationTraitsType>,
        protected QueryEventListener<typename ImplQueryHandler::SimulationTraitsType>,
        // For children to subscribe, but we just ignore their subscriptions,
        // always passing data on ourselves anyway.
        protected LocationUpdateProvider<typename ImplQueryHandler::SimulationTraitsType>
{
    typedef typename ImplQueryHandler::SimulationTraitsType SimulationTraits;
public:
    typedef SimulationTraits SimulationTraitsType;

    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef Query<SimulationTraits> QueryType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;

    typedef typename QueryHandlerType::ShouldTrackCallback ShouldTrackCallback;
    typedef typename QueryHandlerType::ObjectList ObjectList;

    typedef std::tr1::function<ImplQueryHandler*()> ImplConstructor;

    RebuildingQueryHandler(ImplConstructor impl_cons)
     : QueryHandlerType(),
       mImplConstructor(impl_cons),
       mLocCache(NULL),
       mLocUpdateProvider(NULL),
       mStaticObjects(false),
       mShouldTrackCB(0),
       mPrimaryHandler(NULL),
       mRebuildingHandler(NULL)
    {}

    virtual ~RebuildingQueryHandler() {
        delete mPrimaryHandler;
        delete mRebuildingHandler;

        mLocUpdateProvider->removeUpdateListener(this);
    }

    virtual void initialize(LocationServiceCacheType* loc_cache, LocationUpdateProviderType* loc_up_provider, bool static_objects, ShouldTrackCallback should_track_cb = 0) {
        // Save for rebuilding
        mLocCache = loc_cache;
        mLocUpdateProvider = loc_up_provider;
        mLocUpdateProvider->addUpdateListener(this);
        mStaticObjects = static_objects;
        mShouldTrackCB = should_track_cb;

        // Construct
        mPrimaryHandler = mImplConstructor();
        // And pass along
        mPrimaryHandler->initialize(mLocCache, this, mStaticObjects, mShouldTrackCB);
    }

    virtual void addObject(const ObjectID& obj_id) {
        return mPrimaryHandler->addObject(obj_id);
    }
    virtual void removeObject(const ObjectID& obj_id) {
        return mPrimaryHandler->removeObject(obj_id);
    }
    virtual bool containsObject(const ObjectID& obj_id) {
        return mPrimaryHandler->containsObject(obj_id);
    }
    virtual ObjectList allObjects() {
        return mPrimaryHandler->allObjects();
    }

    virtual void tick(const Time& t, bool report = true) {
        mPrimaryHandler->tick(t, report);
    }

    virtual void rebuild() {
    }

    virtual float cost() {
        return mPrimaryHandler->cost();
    }

    virtual uint32 numObjects() const {
        return mPrimaryHandler->numObjects();
    }
    virtual uint32 numQueries() const {
        return mPrimaryHandler->numQueries();
    }

    virtual LocationServiceCacheType* locationCache() const {
        return mLocCache;
    }

    // LocationUpdateListener
    virtual void locationConnected(const ObjectID& obj_id, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) {
        mPrimaryHandler->locationConnected(obj_id, local, pos, region, maxSize);
    }
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        mPrimaryHandler->locationPositionUpdated(obj_id, old_pos, new_pos);
    }
    virtual void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        mPrimaryHandler->locationRegionUpdated(obj_id, old_region, new_region);
    }
    virtual void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) {
        mPrimaryHandler->locationMaxSizeUpdated(obj_id, old_maxSize, new_maxSize);
    }
    virtual void locationDisconnected(const ObjectID& obj_id) {
        mPrimaryHandler->locationDisconnected(obj_id);
    }

    // QueryChangeListener
    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        mImplQueryMap[query]->position(new_pos);
    }
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        mImplQueryMap[query]->region(new_region);
    }
    virtual void queryMaxSizeChanged(QueryType* query, Real old_ms, Real new_ms) {
        mImplQueryMap[query]->maxSize(new_ms);
    }
    virtual void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) {
        mImplQueryMap[query]->angle(new_val);
    }
    virtual void queryDeleted(const QueryType* query) {
        QueryType* inner_query = mImplQueryMap[query];
        delete inner_query;
        mImplQueryMap.erase(query);
        mInvertedQueryMap.erase(inner_query);
    }

protected:
    // LocationUpdateProvider interface. We could track these, but it is easier
    // to just ignore them and always pass the calls on to our children.
    virtual void addUpdateListener(LocationUpdateListenerType* listener) {}
    virtual void removeUpdateListener(LocationUpdateListenerType* listener) {}

    virtual void registerQuery(QueryType* query) {
        QueryType* impl_query = mPrimaryHandler->QueryHandlerType::registerQuery(
            query->position(), query->region(), query->maxSize(), query->angle(), query->radius()
        );

        // Maintain index
        mImplQueryMap[query] = impl_query;
        mInvertedQueryMap[impl_query] = query;

        // Setup listener so we can forward events
        impl_query->setEventListener(this);
    }

    virtual void queryHasEvents(QueryType* query) {
        // Lookup the parent and forward the events
        QueryType* parent_query = mInvertedQueryMap[query];

        std::deque<QueryEventType> evts;
        query->popEvents(evts);
        parent_query->pushEvents(evts);
    }

    ImplConstructor mImplConstructor;
    LocationServiceCacheType* mLocCache;
    LocationUpdateProviderType* mLocUpdateProvider;
    bool mStaticObjects;
    ShouldTrackCallback mShouldTrackCB;

    ImplQueryHandler* mPrimaryHandler;
    ImplQueryHandler* mRebuildingHandler;

    // We need to wrap queries, so we need to maintain a mapping between the
    // query presented externally and the ones from the underlying query
    // handlers.
    typedef std::tr1::unordered_map<const QueryType*, QueryType*> QueryToQueryMap;
    QueryToQueryMap mImplQueryMap; // Our query -> Impl query
    QueryToQueryMap mInvertedQueryMap; // Impl query -> our query

}; // class RebuildingQueryHandler

} // namespace Prox

#endif //_PROX_REBUILDING_QUERY_HANDLER_HPP_
