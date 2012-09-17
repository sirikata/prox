// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_LEVEL_QUERY_HANDLER_TEST_HPP_
#define _LIBPROX_TEST_LEVEL_QUERY_HANDLER_TEST_HPP_

#include <cxxtest/TestSuite.h>
#include "HandlerTestBase.hpp"
#include <prox/geom/LevelQueryHandler.hpp>

// Test of the level query handler, ensuring it returns the expected set of
// results at a given level.
class LevelQueryHandlerTest : public CxxTest::TestSuite,
    public HandlerTestBase<Prox::QueryHandler<Prox::DefaultSimulationTraits>, Prox::Query<Prox::DefaultSimulationTraits> >
{
    typedef HandlerTestBase<Prox::QueryHandler<Prox::DefaultSimulationTraits>, Prox::Query<Prox::DefaultSimulationTraits> > HandlerTestBaseType;
    typedef Prox::LevelQueryHandler<Prox::DefaultSimulationTraits> LevelQueryHandler;
public:
    LevelQueryHandlerTest()
     : HandlerTestBaseType(true)
    {}

    virtual QueryHandler* createHandler() {
        return new LevelQueryHandler(10);
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
        HandlerTestBaseType::addQuery(q);
    }

    void checkCorrectResultsSize(size_t query_idx, size_t nresults) {
        TS_ASSERT_EQUALS(query_results[queries[query_idx]].size(), nresults);
    }

    void setUp() { HandlerTestBaseType::setUp(); }
    void tearDown() { HandlerTestBaseType::tearDown(); }

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
        addQuery(1);
        TS_ASSERT_EQUALS(handler->numQueries(), 1);
        removeQuery(0);
        TS_ASSERT_EQUALS(handler->numQueries(), 0);
    }

    // Test a single queries results as a sanity check that they are working properly.
    void testQueryResults() {
        init(true, false);
        addObjects(0, 100);
        addQuery(0);
        addQuery(1);
        addQuery(2);
        tick();
        // These exact number of results are subject to change based on the tree
        // building approach used. You can verify they are still correct by using
        //handler->draw();
        // to draw the tree and probably turn on one at a time to verify the
        // cuts are in the right place.
        checkCorrectResultsSize(0, 1);
        checkCorrectResultsSize(1, 3);
        checkCorrectResultsSize(2, 19);
    }

    // Test a single queries results as a sanity check that they are working properly.
    void testMaxQueryResults() {
        init(true, false);
        addObjects(0, 100);
        // Add query at level way above the # of levels we should have in the
        // tree
        addQuery(10);
        tick();
        // Should be exact number since it should include all leaf children and
        // only leaf children.
        checkCorrectResultsSize(0, 100);
    }

};

#endif //_LIBPROX_TEST_LEVEL_QUERY_HANDLER_TEST_HPP_
