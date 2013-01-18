// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "GeomHandlerTest.hpp"
#include <prox/geom/RTreeAngleQueryHandler.hpp>
#include "TestSettings.hpp"

class RTreeHandlerTest : public CxxTest::TestSuite, public GeomHandlerTest {
public:
    typedef Prox::RTreeAngleQueryHandler<Prox::DefaultSimulationTraits, TestNodeData> RTreeAngleQueryHandler;

    RTreeHandlerTest()
     : GeomHandlerTest(true)
    {}

    virtual QueryHandler* createHandler() {
        return new RTreeAngleQueryHandler(10);
    }

    void setUp() { GeomHandlerTest::setUp(); }
    void tearDown() { GeomHandlerTest::tearDown(); }

    void testCreate() { GeomHandlerTest::testCreate(); }
    void testAddRemove() { GeomHandlerTest::testAddRemove(); }
    void testAddRemoveMany() { GeomHandlerTest::testAddRemoveMany(); }

    void testAddRemoveQuery() { GeomHandlerTest::testAddRemoveQuery(); }
    void testQueryResults() { GeomHandlerTest::testQueryResults(); }
};
