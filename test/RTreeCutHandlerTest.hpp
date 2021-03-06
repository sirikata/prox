// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "GeomHandlerTest.hpp"
#include <prox/geom/RTreeCutQueryHandler.hpp>
#include "TestSettings.hpp"

class RTreeCutHandlerTest : public CxxTest::TestSuite, public GeomHandlerTest {
public:
    typedef Prox::RTreeCutQueryHandler<Prox::DefaultSimulationTraits, TestNodeData> RTreeCutQueryHandler;

    RTreeCutHandlerTest()
     : GeomHandlerTest(true)
    {}

    virtual QueryHandler* createHandler() {
        return new RTreeCutQueryHandler(10, false);
    }

    void setUp() { GeomHandlerTest::setUp(); }
    void tearDown() { GeomHandlerTest::tearDown(); }

    void testCreate() { GeomHandlerTest::testCreate(); }
    void testAddRemove() { GeomHandlerTest::testAddRemove(); }
    void testAddRemoveMany() { GeomHandlerTest::testAddRemoveMany(); }

    void testAddRemoveQuery() { GeomHandlerTest::testAddRemoveQuery(); }
    void testQueryResults() { GeomHandlerTest::testQueryResults(); }
};
