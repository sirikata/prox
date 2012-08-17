// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_GEOM_HANDLER_TEST_HPP_
#define _LIBPROX_TEST_GEOM_HANDLER_TEST_HPP_

#include <cxxtest/TestSuite.h>
#include <prox/base/QueryCache.hpp>
#include <prox/base/DefaultSimulationTraits.hpp>
#include "TestLocationServiceCache.hpp"
#include <prox/geom/QueryHandler.hpp>
#include <prox/base/QueryEventListener.hpp>
#include <prox/base/AggregateListener.hpp>

// Generic tests for regular geometric (solid angle) query handlers. The
// parameter to the constructor controls whether it's strict or not: strict
// means exact results are computed, e.g. like brute force or rtree, non-strict
// means the results are loosened but conservative, e.g. like
// rtreecut/rtreecutagg, where you'll refine *at least* as far as strict ones,
// but you'll also return some smaller-than-requested results.
class GeomHandlerTest :
    public Prox::QueryEventListener<Prox::DefaultSimulationTraits, Prox::Query<Prox::DefaultSimulationTraits> >,
    public Prox::AggregateListener<Prox::DefaultSimulationTraits>
{
public:
    typedef Prox::QueryHandler<Prox::DefaultSimulationTraits> QueryHandler;
    typedef Prox::Query<Prox::DefaultSimulationTraits> Query;
    typedef Prox::QueryEvent<Prox::DefaultSimulationTraits> QueryEvent;
    typedef Prox::Reference::ObjectID ObjectID;
    typedef Prox::Reference::MotionVector3f MotionVector3f;
    typedef Prox::Reference::Vector3f Vector3f;
    typedef Prox::Reference::BoundingSphere3f BoundingSphere3f;
    typedef Prox::Reference::SolidAngle SolidAngle;
    typedef Prox::float32 float32;
    typedef Prox::uint32 uint32;
    typedef Prox::String String;
    typedef Prox::Reference::Time Time;
    typedef Prox::Reference::Duration Duration;

    // Test objects
    struct ObjectInfo {
        ObjectInfo(
            ObjectID _id,
            MotionVector3f _loc,
            Vector3f _bounds_center,
            float32 _bounds_center_radius,
            float32 _bounds_max_size
        )
         : id(_id), loc(_loc),
           bounds_center(_bounds_center),
           bounds_center_radius(_bounds_center_radius),
           bounds_max_size(_bounds_max_size)
        {}

        ObjectID id;
        MotionVector3f loc;
        Vector3f bounds_center;
        float32 bounds_center_radius;
        float32 bounds_max_size;
    };
    typedef std::vector<ObjectInfo> ObjectList;
    ObjectList objects;

    // Track queries
    typedef std::vector<Query*> QueryList;
    QueryList queries;
    typedef std::map<Query*, size_t> QueryIndexMap;
    QueryIndexMap query_indices;
    // And their current set of results
    typedef std::set<ObjectID> ObjectIDSet;
    typedef std::map<Query*, ObjectIDSet> QueryResultSetMap;
    QueryResultSetMap query_results;

    const bool strict;

    // Per-test data
    TestLocationServiceCache* loccache;
    QueryHandler* handler;
    Time time;

    GeomHandlerTest(bool _strict)
     : strict(_strict),
       loccache(NULL),
       handler(NULL),
       time(Time::null())
    {}

    void setUp() {
        // Set these up in a fixed layout and size so we know which should be
        // returned from queries. We should probably also use other variations
        // to exercise changes in other dimensions. Here we have fixed radius,
        // object-like center-bound (i.e. a point), unit radius, and position
        // only varies along the x-axis.
        for(size_t i = 0; i < 100; i++) {
            objects.push_back(
                ObjectInfo(
                    ObjectID::Random()(),
                    MotionVector3f(Time::null(), Vector3f(i, 0, 0), Vector3f::nil()),
                    Vector3f(0, 0, 0),
                    0, // "object-like" point center-bounds
                    1
                )
            );
        }

        time = Time::null();
    }

    void tearDown() {
        cleanupQueries();

        delete handler; handler = NULL;
        delete loccache; loccache = NULL;

        objects.clear();
        queries.clear();
        query_indices.clear();
    }

    // Subclasses should provide this to create the handler they want to test.
    virtual QueryHandler* createHandler() = 0;

    void init(bool static_objects, bool replicated) {
        loccache = new TestLocationServiceCache();
        handler = createHandler();
        handler->setAggregateListener(this);
        handler->initialize(
            loccache, loccache,
            static_objects, replicated
        );
    }

    void addObject(size_t idx) {
        loccache->addObject(
            objects[idx].id,
            false, // not aggregate
            objects[idx].loc,
            objects[idx].bounds_center,
            objects[idx].bounds_center_radius,
            objects[idx].bounds_max_size,
            "", // no mesh
            true
        );
    }
    void addObjects(size_t start_idx, size_t end_idx) {
        for(size_t i = start_idx; i < end_idx; i++)
            addObject(i);
    }
    void removeObject(size_t idx) {
        loccache->removeObject(
            objects[idx].id
        );
    }
    void removeObjects(size_t start_idx, size_t end_idx) {
        for(size_t i = start_idx; i < end_idx; i++)
            removeObject(i);
    }

    void addQuery(float32 minangle) {
        // For these tests we, always put the querier at the origin, with no
        // bounding sphere, 0 max_size, and just specify the minangle passed in.
        Query* q = handler->registerQuery(
            MotionVector3f(Time::null(), Vector3f::nil(), Vector3f::nil()),
            BoundingSphere3f(Vector3f::nil(), 0.f),
            0,
            SolidAngle(minangle)
        );
        q->setEventListener(this);
        queries.push_back(q);
        query_indices[q] = queries.size()-1;
    }
    void removeQuery(size_t idx) {
        // We never actually remove them from the list so that indices are maintained
        assert(queries[idx] != NULL);
        query_indices.erase(queries[idx]);
        delete queries[idx];
        queries[idx] = NULL;
    }
    void cleanupQueries() {
        for(size_t i = 0; i < queries.size(); i++)
            if (queries[i] != NULL) removeQuery(i);
    }
    bool resultsDoNotContain(Query* query, const ObjectID& objid) {
        return ( query_results[query].find(objid) == query_results[query].end() );
    }
    bool resultsContain(Query* query, const ObjectID& objid) {
        return ( query_results[query].find(objid) != query_results[query].end() );
    }
    virtual void queryHasEvents(Query* query) {
        typedef std::deque<QueryEvent> QueryEventList;
        QueryEventList evts;
        query->popEvents(evts);

        while(!evts.empty()) {
            const QueryEvent& evt = evts.front();

            for(size_t aidx = 0; aidx < evt.additions().size(); aidx++) {
                QueryEvent::Addition add = evt.additions()[aidx];
                TS_ASSERT(resultsDoNotContain(query, add.id()));
                query_results[query].insert(add.id());
            }

            for(size_t ridx = 0; ridx < evt.removals().size(); ridx++) {
                QueryEvent::Removal rem = evt.removals()[ridx];
                TS_ASSERT(resultsContain(query, rem.id()));
                query_results[query].erase(rem.id());
            }

            evts.pop_front();
        }
    }


    void tick() {
        time = time + Duration::milliseconds((float32)100);
        handler->tick(time, false);
    }

    void checkCorrectResultsSize(size_t query_idx, size_t nresults) {
        if (strict) {
            TS_ASSERT_EQUALS(query_results[queries[query_idx]].size(), nresults);
        }
        else {
            TS_ASSERT_LESS_THAN(nresults, query_results[queries[query_idx]].size());
        }
    }


    // The actual tests

    void testCreate() {
        // static objects, not replicated
        init(true, false);
    }

    // Test insertion of objects via the loccache
    void testAddRemove() {
        // static objects, not replicated
        init(true, false);

        TS_ASSERT_EQUALS(handler->numObjects(), 0);
        addObject(0); tick();
        TS_ASSERT_EQUALS(handler->numObjects(), 1);
        removeObject(0); tick();
        TS_ASSERT_EQUALS(handler->numObjects(), 0);
    }

    // Test insertion of objects via the loccache
    void testAddRemoveMany() {
        // static objects, not replicated
        init(true, false);

        addObjects(0, 100); tick();
        TS_ASSERT_EQUALS(handler->numObjects(), 100);
        TS_ASSERT_LESS_THAN_EQUALS(100, handler->numNodes());
        removeObjects(0, 50); tick();
        TS_ASSERT_EQUALS(handler->numObjects(), 50);
        TS_ASSERT_LESS_THAN_EQUALS(50, handler->numNodes());
        removeObjects(50, 100); tick();
        TS_ASSERT_EQUALS(handler->numObjects(), 0);
    }

    void testAddRemoveQuery() {
        init(true, false);
        TS_ASSERT_EQUALS(handler->numQueries(), 0);
        addQuery(.1);
        TS_ASSERT_EQUALS(handler->numQueries(), 1);
        removeQuery(0);
        TS_ASSERT_EQUALS(handler->numQueries(), 0);
    }

    // Test a single queries results as a sanity check that they are working properly.
    void testQueryResults() {
        init(true, false);
        addObjects(0, 100);
        addQuery(.01);
        addQuery(.001);
        tick();
        checkCorrectResultsSize(0, 19);
        checkCorrectResultsSize(1, 58);
    }

    // Test that queries get proper updates when objects are added and removed
    // with active queries
    void testQueryResultsUpdated() {
        init(true, false);
        // Baseline of objects added before queries
        addObjects(0, 10);
        addQuery(.01);
        addQuery(.001);
        tick();
        checkCorrectResultsSize(0, 10);
        checkCorrectResultsSize(1, 10);

        // Add rest of objects, should match results of previous test
        addObjects(10, 100);
        tick();
        checkCorrectResultsSize(0, 19);
        checkCorrectResultsSize(1, 58);

        // Remove enough to affect only the second query
        removeObjects(25, 100);
        tick();
        checkCorrectResultsSize(0, 19);
        checkCorrectResultsSize(1, 25);

        // And remove the rest
        removeObjects(0, 25);
        tick();
        checkCorrectResultsSize(0, 0);
        checkCorrectResultsSize(1, 0);
    }


    // NOTE that as you add tests, you need to provide the implementations in
    // subclasses because of the way cxxtest works.


    // AggregateListener Interface -- for query handlers that return aggregates
    // we need to add/remove aggregates to the loccache. Normally these would
    // arrive via the LocationService, but we just push them in directly here to
    // avoid including more pieces.
    virtual void aggregateCreated(AggregatorType* handler, const ObjectIDType& objid) {
        loccache->addObject(
            objid,
            true, // aggregate
            MotionVector3f(Time::null(), Vector3f(0, 0, 0), Vector3f::nil()),
            Vector3f(0, 0, 0), 0, 0,
            "", // no mesh
            false
        );
    }
    virtual void aggregateChildAdded(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) {
    }
    virtual void aggregateChildRemoved(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) {
    }
    virtual void aggregateBoundsUpdated(AggregatorType* handler, const ObjectIDType& objid,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) {
        loccache->updateBounds(objid, bnds_center_offset, bnds_center_bounds_radius, bnds_max_object_size);
    }
    virtual void aggregateDestroyed(AggregatorType* handler, const ObjectIDType& objid) {
        loccache->removeObject(objid);
    }
    virtual void aggregateObserved(AggregatorType* handler, const ObjectIDType& objid, uint32 nobservers) {}

};

#endif //_LIBPROX_TEST_GEOM_HANDLER_TEST_HPP_
