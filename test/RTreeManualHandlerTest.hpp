// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_RTREE_MANUAL_HANDLER_TEST_HPP_
#define _LIBPROX_TEST_RTREE_MANUAL_HANDLER_TEST_HPP_

#include <cxxtest/TestSuite.h>
#include "HandlerTestBase.hpp"
#include <prox/manual/RTreeManualQueryHandler.hpp>

// Test of just the functionality of the manual query handler given a known
// replicated tree.
class RTreeManualHandlerTest : public CxxTest::TestSuite,
    public HandlerTestBase<Prox::ManualQueryHandler<Prox::DefaultSimulationTraits>, Prox::ManualQuery<Prox::DefaultSimulationTraits> >
{
    typedef HandlerTestBase<Prox::ManualQueryHandler<Prox::DefaultSimulationTraits>, Prox::ManualQuery<Prox::DefaultSimulationTraits> > HandlerTestBaseType;
public:

    RTreeManualHandlerTest()
     : HandlerTestBaseType(true) // aggregates added to loc since this query
                                 // handler works over just input set of objects
    {}

    virtual QueryHandler* createHandler() {
        return new Prox::RTreeManualQueryHandler<Prox::DefaultSimulationTraits>(10);
    }

    void addQuery() {
        Query* q = handler->registerQuery(
            MotionVector3f(Time::null(), Vector3f::nil(), Vector3f::nil()),
            BoundingSphere3f(Vector3f::nil(), 0.f),
            0
        );
        HandlerTestBaseType::addQuery(q);
    }


    void setUp() { HandlerTestBaseType::setUp(); }
    void tearDown() { HandlerTestBaseType::tearDown(); }

    size_t resultsSize(size_t query_idx) {
        return query_results[queries[query_idx]].size();
    }

    void testCreate() {
        // static objects, not replicated
        init(true, false);
    }

    // Test that basic refinement works properly
    void testSimpleRefineCoarsen() {
        init(true, false);
        addObjects(0, 100);
        addQuery();
        TS_ASSERT_EQUALS(resultsSize(0), 1);

        ObjectID root = *(query_results[queries[0]].begin()), root_child = ObjectID::Random()();
        queries[0]->refine(root);
        // Check that all children of the node were added
        size_t nchildren = 0;
        for(QueryHandler::NodeIterator nit = handler->nodesBegin(); nit != handler->nodesEnd(); nit++) {
            if (nit.parentId() == root) {
                TS_ASSERT(resultsContain(queries[0], nit.id()));
                nchildren++;
                // Track one of these for coarsening
                root_child = nit.id();
            }
        }
        // and we shouldn't have any more nodes than just the root + it's
        // children
        TS_ASSERT_EQUALS(resultsSize(0), nchildren+1);

        queries[0]->coarsen(root_child);
        // Should be back to just root.
        TS_ASSERT_EQUALS(resultsSize(0), 1);
        TS_ASSERT(resultsContain(queries[0], root));
    }

    // Test transition to/from empty tree
    void testEmptyTree() {
        init(true, false);
        addQuery();

        // We should be at 0 with no objects in the tree
        TS_ASSERT_EQUALS(resultsSize(0), 0);
        addObjects(0, 100);
        // Then we should be non-zero
        TS_ASSERT_LESS_THAN(0, resultsSize(0));
        removeObjects(0, 100);
        // And keeps the node after removal
        TS_ASSERT_EQUALS(resultsSize(0), 1);
    }

};

#endif //_LIBPROX_TEST_RTREE_MANUAL_HANDLER_TEST_HPP_
