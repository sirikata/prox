/*  libprox
 *  QueryHandler.hpp
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

#ifndef _PROX_QUERY_HANDLER_HPP_
#define _PROX_QUERY_HANDLER_HPP_

#include <prox/Query.hpp>
#include <prox/LocationServiceCache.hpp>
#include <prox/DefaultSimulationTraits.hpp>
#include <prox/AggregateListener.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class QueryHandler : public LocationUpdateListener<SimulationTraits>, public QueryChangeListener<SimulationTraits> {
public:
    typedef SimulationTraits SimulationTraitsType;

    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;
    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef Query<SimulationTraits> QueryType;
    typedef typename QueryType::ID QueryID;

    // Signature for callback allowing the user to control whether an object is
    // actually added or not.
    typedef std::tr1::function<bool(const ObjectID& obj_id, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize)> ShouldTrackCallback;

    typedef std::vector<LocCacheIterator> ObjectList;

    QueryHandler()
     : LocationUpdateListenerType(),
       QueryChangeListenerType(),
       mQueryIDSource(0),
       mAggregateListener(NULL),
       mTrackChecks(false),
       mShouldRestructure(false),
       mReportRestructures(false),
       mReportHealth(false),
       mReportHealthFrequency(1),
       mReportCost(false),
       mItsSinceReportedHealth(0),
       mReportQueryStats(false)
    {}
    virtual ~QueryHandler() {}


    void trackChecks(bool t) { mTrackChecks = t; }
    void shouldRestructure(bool r) { mShouldRestructure = r; }
    void reportRestructures(bool r) { mReportRestructures = r; }
    bool reportRestructures() const { return mReportRestructures; }
    void reportHealth(bool r) { mReportHealth = r; }
    void reportHealthFrequency(int its) { mReportHealthFrequency = its; }
    void reportCost(bool r) { mReportCost = r; }
    void reportQueryStats(bool r) { mReportQueryStats = r; }

    /** Initialze the query handler.
     *  \param loc_cache LocationServiceCache to use for learning about object
     *                   updates
     *  \param static_objects if true, disables processing necessary for
     *                        supporting dynamic objects.
     *  \param should_track_cb if non-NULL, invoked when new objects are
     *                         discovered to decide whether to track them and
     *                         return them to queriers.
     */
    virtual void initialize(LocationServiceCacheType* loc_cache, LocationUpdateProviderType* loc_up_provider, bool static_objects, ShouldTrackCallback should_track_cb = 0) = 0;

    /** Add an object to be considered in the result set.  This method, along
     *  with ShouldTrackCallback and removeObject, allows the user to control
     *  which subset of objects from the location service cache are candidate
     *  results for this handler.
     *
     *  Should only be invoked with ObjectID's of objects completely added to
     *  the location service cache, i.e. not newly connected objects.  This
     *  means it should *not* be invoked in
     *  LocationUpdateListener::locationConnected. Use ShouldTrackCallback for
     *  that case, and addObject when an object was removed and should now be
     *  added back again.
     */
    virtual void addObject(const ObjectID& obj_id) = 0;
    virtual void addObject(const LocCacheIterator& obj_loc_it) = 0;
    // These versions allow use to get bind() to disambiguate
    void addObjectByID(const ObjectID& obj_id) {
        addObject(obj_id);
    }
    void addObjectByLocCacheIt(const LocCacheIterator& obj_loc_it) {
        addObject(obj_loc_it);
    }
    /** Remove an object from consideration. If called with an object not in the
     *  tree, will be ignored. This method, along with addObject and
     *  ShouldTrackCallback, allows the user to control which subset of objects
     *  from the location service cache are candidate results for this handler. */
    virtual void removeObject(const ObjectID& obj_id) = 0;
    /** Checks if this handler is currently tracking the given object. */
    virtual bool containsObject(const ObjectID& obj_id) = 0;
    /** Get a list of the current objects tracked. Used for collecting the
     * information necessary to rebuild a data structure. LocCacheIterators are
     * extracted rather than IDs to ensure startTracking() has been called on
     * each of the objects: this ensures the objects remain alive during their
     * use and the caller is responsible for calling stopTracking.
     */
    virtual ObjectList allObjects() = 0;

    /** Bulk load objects. MUST NOT start tracking the objects in the location
     *  cache -- a requirement of calling this method is that that should have
     *  already been done in the correct thread, and the results of doing so are
     *  the parameter to this function. Defaults to the naive wrapper around addObject,
     *  but may be overridden with a more efficient implementation.
     *  There must be no existing objects in the QueryHandler.
     */
    virtual void bulkLoad(const ObjectList& objs) {
        for(typename ObjectList::const_iterator it = objs.begin(); it != objs.end(); it++)
            addObject(*it);
    }

    QueryType* registerQuery(const MotionVector3& pos, const BoundingSphere& region, Real maxSize, const SolidAngle& minAngle) {
        QueryType* q = new QueryType(this, mQueryIDSource++, pos, region, maxSize, minAngle);
        registerQuery(q);
        return q;
    }
    QueryType* registerQuery(const MotionVector3& pos, const BoundingSphere& region, Real maxSize, const SolidAngle& minAngle, float radius) {
        QueryType* q = new QueryType(this, mQueryIDSource++, pos, region, maxSize, minAngle, radius);
        registerQuery(q);
        return q;
    }

    virtual void tick(const Time& t, bool report = true) = 0;

    virtual void rebuild() = 0;

    /** Get an estimation of the cost of evaluating a query on this query
     *  handler.
     */
    virtual float cost() = 0;

    virtual uint32 numObjects() const = 0;
    virtual uint32 numQueries() const = 0;

    void setAggregateListener(AggregateListenerType* listener) {
        mAggregateListener = listener;
    }
    void removeAggregateListener() {
        mAggregateListener = NULL;
    }

    virtual LocationServiceCacheType* locationCache() const = 0;

    // LocationUpdateListener
    virtual void locationConnected(const ObjectID& obj_id, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) = 0;
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) = 0;
    virtual void locationDisconnected(const ObjectID& obj_id) = 0;

    // QueryChangeListener
    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void queryMaxSizeChanged(QueryType* query, Real old_ms, Real new_ms) = 0;
    virtual void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) = 0;
    virtual void queryDestroyed(QueryType* query, bool implicit) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;

protected:
    virtual void registerQuery(QueryType* query) = 0;

    QueryID mQueryIDSource;
    AggregateListenerType* mAggregateListener;

    // Whether to track constraint checks
    bool mTrackChecks;
    bool mShouldRestructure;
    bool mReportRestructures;
    bool mReportHealth;
    int mReportHealthFrequency;
    bool mReportCost;
    int mItsSinceReportedHealth;
    bool mReportQueryStats;
}; // class QueryHandler

} // namespace Prox

#endif //_PROX_QUERY_HANDLER_HPP_
