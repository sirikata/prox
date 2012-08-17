// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_GEOM_HANDLER_TEST_HPP_
#define _LIBPROX_TEST_GEOM_HANDLER_TEST_HPP_

#include <cxxtest/TestSuite.h>
#include "HandlerTestBase.hpp"
#include <prox/geom/QueryHandler.hpp>

// Generic tests for regular geometric (solid angle) query handlers. The
// parameter to the constructor controls whether it's strict or not: strict
// means exact results are computed, e.g. like brute force or rtree, non-strict
// means the results are loosened but conservative, e.g. like
// rtreecut/rtreecutagg, where you'll refine *at least* as far as strict ones,
// but you'll also return some smaller-than-requested results.
class GeomHandlerTest :
    public HandlerTestBase<Prox::QueryHandler<Prox::DefaultSimulationTraits>, Prox::Query<Prox::DefaultSimulationTraits> >
{
    typedef HandlerTestBase<Prox::QueryHandler<Prox::DefaultSimulationTraits>, Prox::Query<Prox::DefaultSimulationTraits> > HandlerTestBaseType;
public:
    const bool strict;

    GeomHandlerTest(bool _strict)
     : HandlerTestBaseType(),
       strict(_strict)
    {}

    void checkCorrectResultsSize(size_t query_idx, size_t nresults) {
        if (strict) {
            TS_ASSERT_EQUALS(query_results[queries[query_idx]].size(), nresults);
        }
        else {
            TS_ASSERT_LESS_THAN(nresults, query_results[queries[query_idx]].size());
        }
    }

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
};

#endif //_LIBPROX_TEST_GEOM_HANDLER_TEST_HPP_
