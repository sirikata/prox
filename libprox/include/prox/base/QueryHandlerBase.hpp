// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_BASE_QUERY_HANDLER_HPP_
#define _PROX_BASE_QUERY_HANDLER_HPP_

#include <prox/base/LocationServiceCache.hpp>
#include <prox/base/Aggregator.hpp>

#include <prox/base/impl/NodeIterator.hpp>

namespace Prox {

template<typename SimulationTraits, typename QueryTypeT, typename QueryChangeListenerTypeT>
class QueryHandlerBase :
        public LocationUpdateListener<SimulationTraits>,
        public QueryChangeListenerTypeT,
        public Aggregator<SimulationTraits>
{
public:
    typedef SimulationTraits SimulationTraitsType;

    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef LocationUpdateProvider<SimulationTraits> LocationUpdateProviderType;
    typedef QueryChangeListenerTypeT QueryChangeListenerType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::UniqueIDGeneratorType UniqueIDGenerator;
    typedef QueryTypeT QueryType;
    typedef typename QueryType::ID QueryID;

    // Signature for callback allowing the user to control whether an object is
    // actually added or not.
    typedef std::tr1::function<bool(const ObjectID& obj_id, bool local, bool aggregate, const MotionVector3& pos, const BoundingSphere& region, Real maxSize)> ShouldTrackCallback;

    typedef std::vector<LocCacheIterator> ObjectList;

    typedef QueryHandlerBaseImpl::NodeIterator<SimulationTraits> NodeIterator;
    typedef QueryHandlerBaseImpl::NodeIteratorImpl<SimulationTraits> NodeIteratorImpl;

    QueryHandlerBase()
     : LocationUpdateListenerType(),
       QueryChangeListenerType(),
       mHandlerIndexID( UniqueIDGenerator()() )
    {}
    virtual ~QueryHandlerBase() {}

    /** Initialze the query handler.
     *  \param loc_cache LocationServiceCache to use for learning about object
     *                   updates
     *  \param loc_up_provider LocationUpdateProvider for location updates.
     *  \param static_objects if true, disables processing necessary for
     *                        supporting dynamic objects.
     *  \param replicated if true, this query handler will use a data structure
     *                    replicated from another node.
     *  \param should_track_cb if non-NULL, invoked when new objects are
     *                         discovered to decide whether to track them and
     *                         return them to queriers.
     *
     *  Note: Although LocationServiceCache implements LocationUpdateProvider,
     *  it is a pain to have to reimplement all of the LocationServiceCache just
     *  to override LocationUpdateProvider. This split is rarely used. For an
     *  example, see RebuildingQueryHandler, which ignores
     *  LocationUpdateListener subscriptions and always passes events onto its
     *  child handlers.
     */
    virtual void initialize(LocationServiceCacheType* loc_cache, LocationUpdateProviderType* loc_up_provider, bool static_objects, bool replicated, ShouldTrackCallback should_track_cb = 0) = 0;

    /** Returns true if this query handler is configured to only handle static
     *  objects.
     */
    virtual bool staticOnly() const = 0;

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
    virtual void addObject(const ObjectID& obj_id, const ObjectID& parent_id) = 0;
    virtual void addObject(const LocCacheIterator& obj_loc_it) = 0;
    // These versions allow use to get bind() to disambiguate
    void addObjectByID(const ObjectID& obj_id) {
        addObject(obj_id);
    }
    void addObjectByIDWithParent(const ObjectID& obj_id, const ObjectID& parent_id) {
        addObject(obj_id, parent_id);
    }
    void addObjectByLocCacheIt(const LocCacheIterator& obj_loc_it) {
        addObject(obj_loc_it);
    }
    /** Remove an object from consideration. If called with an object not in the
     *  tree, will be ignored. This method, along with addObject and
     *  ShouldTrackCallback, allows the user to control which subset of objects
     *  from the location service cache are candidate results for this handler.
     *
     *  \param obj_id the object to remove
     *  \param temporary if true, treat this removal as temporary. This causes
     *  any removal events triggered by it to be marked as temporary. Useful if
     *  you have multiple query handlers and are moving objects between them (as
     *  opposed to an object actually being deleted).
     */
    virtual void removeObject(const ObjectID& obj_id, bool temporary = false) = 0;
    /** Checks if this handler is currently tracking the given object. */
    virtual bool containsObject(const ObjectID& obj_id) = 0;
    /** Get a list of the current objects tracked. Used for collecting the
     * information necessary to rebuild a data structure. LocCacheIterators are
     * extracted rather than IDs to ensure startTracking() has been called on
     * each of the objects: this ensures the objects remain alive during their
     * use and the caller is responsible for calling stopTracking.
     */
    virtual ObjectList allObjects() = 0;

    virtual void addNode(const ObjectID& objid, const ObjectID& parentid) = 0;
    virtual void removeNode(const ObjectID& objid, bool temporary = false) = 0;

    virtual void reparent(const ObjectID& objid, const ObjectID& parentid) = 0;

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


    virtual void tick(const Time& t, bool report = true) = 0;

    virtual uint32 numObjects() const = 0;
    virtual uint32 numQueries() const = 0;
    virtual uint32 numNodes() const = 0;

    virtual LocationServiceCacheType* locationCache() const = 0;

    // Iterators over 'nodes', which are whatever internal representation is
    // used for objects and collections of objects. Sometimes this is just the
    // same as the set of all objects, other times it may include aggregates.
    NodeIterator nodesBegin() const { return NodeIterator(nodesBeginImpl()); };
    NodeIterator nodesEnd() const { return NodeIterator(nodesEndImpl()); };

    // LocationUpdateListener
    virtual void locationConnected(const ObjectID& obj_id, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) = 0;
    virtual void locationConnectedWithParent(const ObjectID& obj_id, const ObjectID& parent, bool aggregate, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) = 0;
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) = 0;
    virtual void locationDisconnected(const ObjectID& obj_id, bool temporary = false) = 0;

    // QueryChangeListener
    virtual void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void queryRegionChanged(QueryType* query, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void queryMaxSizeChanged(QueryType* query, Real old_ms, Real new_ms) = 0;
    virtual void queryDestroyed(QueryType* query, bool implicit) = 0;
    virtual void queryDeleted(const QueryType* query) = 0;

    /** Get the handlerID for this query handler. Beware that there may be
     *  multiple underlying handler IDs, any of which may be reported
     *  to ensure only a single tree is required to replicate an
     *  entire handler ID's data. Only use this if you know what
     *  you're doing.
     */
    QueryHandlerIndexID handlerID() const { return mHandlerIndexID; }

    /** Draw the data structure for this query handler. Useful for debugging. */
    virtual void draw() { }
protected:

    // Implementation of iterators
    virtual NodeIteratorImpl* nodesBeginImpl() const = 0;
    virtual NodeIteratorImpl* nodesEndImpl() const = 0;

private:
    QueryHandlerIndexID mHandlerIndexID;
}; // class QueryHandlerBase

} // namespace Prox

#endif //_PROX_BASE_QUERY_HANDLER_HPP_
