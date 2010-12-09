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
#include <boost/thread.hpp>

namespace Prox {

/** RebuildingQueryHandler is a relatively small wrapper around a regular
 *  QueryHandler which manages the rebuilding process using two separate
 *  QueryHandlers. The rebuilding process is asynchronous and allows processing
 *  to continue, including addition and removal of objects and queries, while
 *  the rebuilding is in progress.
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class RebuildingQueryHandler :
        public QueryHandler<SimulationTraits>,
        protected QueryEventListener<SimulationTraits>,
        // For children to subscribe, but we just ignore their subscriptions,
        // always passing data on ourselves anyway.
        protected LocationUpdateProvider<SimulationTraits>
{
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

    typedef std::tr1::function<QueryHandlerType*()> ImplConstructor;

    RebuildingQueryHandler(ImplConstructor impl_cons, int batch_size)
     : QueryHandlerType(),
       mImplConstructor(impl_cons),
       mLocCache(NULL),
       mLocUpdateProvider(NULL),
       mStaticObjects(false),
       mShouldTrackCB(0),
       mPrimaryHandler(NULL),
       mRebuildingHandler(NULL),
       mState(IDLE),
       mRebuildObjectList(),
       mRebuildingMutex(),
       mRebuildRequest(),
       mRebuildThread(&RebuildingQueryHandler::asyncRebuildThread, this),
       mMaxQueriesTransitionedPerIteration(batch_size)
    {}

    virtual ~RebuildingQueryHandler() {
        // Allow other thread to exit
        setRebuildingState(EXITING);
        mRebuildRequest.notify_one();
        mRebuildThread.join();

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
        using std::tr1::placeholders::_1;
        mPrimaryHandler->addObject(obj_id);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::addObject, _1, obj_id));
        else if (mustDuplicate())
            mRebuildingHandler->addObject(obj_id);
    }
    virtual void removeObject(const ObjectID& obj_id) {
        using std::tr1::placeholders::_1;
        mPrimaryHandler->removeObject(obj_id);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::removeObject, _1, obj_id));
        else if (mustDuplicate())
            mRebuildingHandler->removeObject(obj_id);
    }
    virtual bool containsObject(const ObjectID& obj_id) {
        if (mPrimaryHandler->containsObject(obj_id))
            return true;
        if (mustDuplicate() && mRebuildingHandler->containsObject(obj_id))
            return true;
        return false;
    }

    virtual ObjectList allObjects() {
        ObjectList results = mPrimaryHandler->allObjects();
        if (mustDuplicate()) {
            ObjectList second_results = mRebuildingHandler->allObjects();
            results.insert(results.begin(), second_results.begin(), second_results.end());
        }
        return results;
    }

    virtual void tick(const Time& t, bool report = true) {
        if (mState == BEGIN_MOVING_QUERIES)
            startSwappingQueries();

        if (mState == MOVING_QUERIES)
            swapSomeQueries();

        mPrimaryHandler->tick(t, report);
        if (mustDuplicate())
            mRebuildingHandler->tick(t, report);
    }

    virtual void rebuild() {
        if (mState != IDLE) return;

        beginAsyncRebuild();
    }

    virtual float cost() {
        // Cost always considers the primary handler -- its not
        // obvious how to combine the costs and we're targetting the
        // primary handler anyway.
        return mPrimaryHandler->cost();
    }

    virtual uint32 numObjects() const {
        return mPrimaryHandler->numObjects() + (mustDuplicate() ? mRebuildingHandler->numObjects() : 0);
    }
    virtual uint32 numQueries() const {
        return mPrimaryHandler->numQueries() + (mustDuplicate() ? mRebuildingHandler->numQueries() : 0);
    }

    virtual LocationServiceCacheType* locationCache() const {
        return mLocCache;
    }

    // LocationUpdateListener
    virtual void locationConnected(const ObjectID& obj_id, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) {
        using std::tr1::placeholders::_1;
        mPrimaryHandler->locationConnected(obj_id, local, pos, region, maxSize);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::locationConnected, _1, obj_id, local, pos, region, maxSize));
        else if (mustDuplicate())
            mRebuildingHandler->locationConnected(obj_id, local, pos, region, maxSize);
    }
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        using std::tr1::placeholders::_1;
        mPrimaryHandler->locationPositionUpdated(obj_id, old_pos, new_pos);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::locationPositionUpdated, _1, obj_id, old_pos, new_pos));
        else if (mustDuplicate())
            mRebuildingHandler->locationPositionUpdated(obj_id, old_pos, new_pos);

    }
    virtual void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) {
        using std::tr1::placeholders::_1;
        mPrimaryHandler->locationRegionUpdated(obj_id, old_region, new_region);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::locationRegionUpdated, _1, obj_id, old_region, new_region));
        else if (mustDuplicate())
            mRebuildingHandler->locationRegionUpdated(obj_id, old_region, new_region);
    }
    virtual void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) {
        using std::tr1::placeholders::_1;
        mPrimaryHandler->locationMaxSizeUpdated(obj_id, old_maxSize, new_maxSize);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::locationMaxSizeUpdated, _1, obj_id, old_maxSize, new_maxSize));
        else if (mustDuplicate())
            mRebuildingHandler->locationMaxSizeUpdated(obj_id, old_maxSize, new_maxSize);
    }
    virtual void locationDisconnected(const ObjectID& obj_id) {
        using std::tr1::placeholders::_1;
        mPrimaryHandler->locationDisconnected(obj_id);
        if (mustDefer())
            mDeferredOperations.push(std::tr1::bind(&QueryHandlerType::locationDisconnected, _1, obj_id));
        else if (mustDuplicate())
            mRebuildingHandler->locationDisconnected(obj_id);
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
    virtual void queryDestroyed(QueryType* query, bool implicit) {
        typename QueryToQueryMap::iterator it = mImplQueryMap.find(const_cast<QueryType*>(query));
        assert( it != mImplQueryMap.end() );
        QueryType* inner_query = it->second;

        // If necessary, destroy inner query manually to pick up closing events
        if (!implicit) {
            inner_query->destroy();
            moveQueryEvents(inner_query, query);
        }

        delete inner_query;
        mImplQueryMap.erase(it);
        mInvertedQueryMap.erase(inner_query);
    }
    virtual void queryDeleted(const QueryType* query) {
    }

protected:
    // Helper to move queries from implementation query to user's query
    void moveQueryEvents(QueryType* from_query, QueryType* to_query) {
        std::deque<QueryEventType> evts;
        from_query->popEvents(evts);
        to_query->pushEvents(evts);
    }


    // LocationUpdateProvider interface. We could track these, but it is easier
    // to just ignore them and always pass the calls on to our children.
    virtual void addUpdateListener(LocationUpdateListenerType* listener) {}
    virtual void removeUpdateListener(LocationUpdateListenerType* listener) {}

    virtual void registerQuery(QueryType* query) {
        registerQuery(query, true);
    }
    virtual void registerQuery(QueryType* query, bool is_new) {
        if (is_new)
            query->addChangeListener(this);

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
        assert(mInvertedQueryMap.find(query) != mInvertedQueryMap.end());

        QueryType* parent_query = mInvertedQueryMap[query];
        moveQueryEvents(query, parent_query);
    }


    // The following methods implement the core of the async
    // rebuilding algorithm.

    void beginAsyncRebuild() {
        setRebuildingState(REBUILDING);

        mRebuildObjectList = mPrimaryHandler->allObjects();
        mRebuildRequest.notify_one();
    }

    // Handles the actual rebuilding.
    void asyncRebuildThread() {
        while(true) {
            // Wait for rebuild request
            Lock lck(mRebuildingMutex);
            mRebuildRequest.wait(lck);

            if (exiting()) return;

            mRebuildingHandler = mImplConstructor();
            mRebuildingHandler->initialize(mLocCache, this, mStaticObjects, mShouldTrackCB);
            mRebuildingHandler->bulkLoad(mRebuildObjectList);
            mRebuildObjectList.clear();

            // Process continues when main thread picks up that this flag was
            // triggered
            setRebuildingState(BEGIN_MOVING_QUERIES);

            // Wait until main thread says its done with deferred handler and we
            // can clear it out
            mRebuildRequest.wait(lck);

            if (exiting()) return;

            assert(mState == DELETING_OLD_HANDLER);

            // Clear out the old handler, now in the rebuilding handler pointer
            delete mRebuildingHandler;
            mRebuildingHandler = NULL;

            setRebuildingState(IDLE);
        }
    }

    void startSwappingQueries() {
        // At this stage we have both trees built, but the queries are all on
        // the old one. We need to transition the queries over, swap the trees,
        // and handle any outstanding object additions, updates, and removals.

        setRebuildingState(MOVING_QUERIES);

        // Run deferred operations on the new tree
        while(!mDeferredOperations.empty()) {
            mDeferredOperations.front()(mRebuildingHandler);
            mDeferredOperations.pop();
        }

        mUntransitionedQueries = mImplQueryMap;
        mQueryTransitionIt = mUntransitionedQueries.begin();

        // Swap the query handlers. This allows us to use registerQuery to get
        // new queries into the right query handler
        std::swap(mPrimaryHandler, mRebuildingHandler);
    }

    void swapSomeQueries() {
        // Move queries to new handler
        int count = 0;
        while (mQueryTransitionIt != mUntransitionedQueries.end() && count < mMaxQueriesTransitionedPerIteration) {
            QueryType* real_query = mQueryTransitionIt->first;
            QueryType* slave_query = mQueryTransitionIt->second;

            // The query may have been removed since we grabbed this
            // information, check that it is still valid.
            if (mImplQueryMap.find(real_query) == mImplQueryMap.end()) {
                mQueryTransitionIt++;
                continue;
            }

            // Remove old maps for this query
            assert(mImplQueryMap.find(real_query) != mImplQueryMap.end());
            assert(mInvertedQueryMap.find(slave_query) != mImplQueryMap.end());
            mImplQueryMap.erase(real_query);
            mInvertedQueryMap.erase(slave_query);

            // Destroy in two phases. First do simple destruction so we get
            // removal events for everything left in the query.
            slave_query->destroy();
            moveQueryEvents(slave_query, real_query);
            // Then actually delete the query since we don't want it anymore
            delete slave_query;

            // And finally, register the new one to start picking up new results.
            registerQuery(real_query, false);

            mQueryTransitionIt++;
            count++;
        }

        // When we finish processing all these, trigger the next stage
        // of processing -- deleting the old
        if (mQueryTransitionIt == mUntransitionedQueries.end()) {
            mUntransitionedQueries.clear();
            mQueryTransitionIt = mUntransitionedQueries.end();

            setRebuildingState(DELETING_OLD_HANDLER);
            // Signal other thread to clean out old handler and mark transition as finished
            mRebuildRequest.notify_one();
        }
    }

    ImplConstructor mImplConstructor;
    LocationServiceCacheType* mLocCache;
    LocationUpdateProviderType* mLocUpdateProvider;
    bool mStaticObjects;
    ShouldTrackCallback mShouldTrackCB;

    QueryHandlerType* mPrimaryHandler;
    QueryHandlerType* mRebuildingHandler;

    // We need to wrap queries, so we need to maintain a mapping between the
    // query presented externally and the ones from the underlying query
    // handlers.
    typedef std::tr1::unordered_map<QueryType*, QueryType*> QueryToQueryMap;
    QueryToQueryMap mImplQueryMap; // Our query -> Impl query
    QueryToQueryMap mInvertedQueryMap; // Impl query -> our query

    typedef boost::thread Thread;
    typedef boost::mutex Mutex;
    typedef boost::unique_lock<Mutex> Lock;
    typedef boost::condition_variable CondVar;

    enum RebuildingState {
        IDLE,
        REBUILDING,
        BEGIN_MOVING_QUERIES,
        MOVING_QUERIES,
        DELETING_OLD_HANDLER,
        EXITING
    };

    void setRebuildingState(RebuildingState s) {
        if (mState == EXITING) return;
        mState = s;
    }

    bool mustDefer() const {
        return (mState == REBUILDING || mState == BEGIN_MOVING_QUERIES);
    }
    bool mustDuplicate() const {
        return (mState == MOVING_QUERIES);
    }
    bool exiting() const {
        return (mState == EXITING);
    }

    RebuildingState mState;
    ObjectList mRebuildObjectList;
    Mutex mRebuildingMutex;
    CondVar mRebuildRequest;
    // Note that this is last to ensure initialization order
    Thread mRebuildThread;

    typedef std::tr1::function<void(QueryHandlerType*)> DeferredOperation;
    std::queue<DeferredOperation> mDeferredOperations;

    QueryToQueryMap mUntransitionedQueries;
    typename QueryToQueryMap::iterator mQueryTransitionIt;

    int mMaxQueriesTransitionedPerIteration;
}; // class RebuildingQueryHandler

} // namespace Prox

#endif //_PROX_REBUILDING_QUERY_HANDLER_HPP_
