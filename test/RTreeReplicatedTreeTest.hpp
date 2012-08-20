// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_RTREE_REPLICATED_TREE_TEST_HPP_
#define _LIBPROX_TEST_RTREE_REPLICATED_TREE_TEST_HPP_

#include <cxxtest/TestSuite.h>
#include "HandlerTestBase.hpp"
#include <prox/manual/RTreeManualQueryHandler.hpp>

// Test of tree replication into a manual RTree query handler
class RTreeReplicatedTreeTest : public CxxTest::TestSuite,
    public HandlerTestBase<Prox::ManualQueryHandler<Prox::DefaultSimulationTraits>, Prox::ManualQuery<Prox::DefaultSimulationTraits> >
{
    typedef HandlerTestBase<Prox::ManualQueryHandler<Prox::DefaultSimulationTraits>, Prox::ManualQuery<Prox::DefaultSimulationTraits> > HandlerTestBaseType;
public:

    RTreeReplicatedTreeTest()
     : HandlerTestBaseType(false) // aggregtes not added to loc since they are
                                  // replicated already
    {}

    static ObjectID GenerateObjectID(size_t id) {
        char raw[ObjectID::static_size];
        memset(raw, 0, ObjectID::static_size);
        ((size_t*)raw)[0] = id;
        return ObjectID(raw, ObjectID::static_size);
    }

    struct Node {
        Node()
         : id(ObjectID::null()), parent(ObjectID::null())
        {}
        Node(const ObjectID& _id, const ObjectID& _parent, bool _leaf)
         : id(_id), parent(_parent), is_leaf(_leaf)
        {}

        ObjectID id;
        ObjectID parent;
        bool is_leaf;
    };
    typedef std::vector<Node> NodeList;
    typedef std::map<ObjectID, Node> NodeMap;

    virtual QueryHandler* createHandler() {
        return new Prox::RTreeManualQueryHandler<Prox::DefaultSimulationTraits>(10);
    }

    void setUp() {
        HandlerTestBaseType::setUp();

        // Construct a simple list of nodes/leaves we can work with/verify against
    }
    void tearDown() { HandlerTestBaseType::tearDown(); }

    void addReplicatedObject(const Node& n) {
        loccache->addObjectWithParent(
            n.id,
            n.parent,
            !n.is_leaf,
            MotionVector3f(Time::null(), Vector3f(0, 0, 0), Vector3f::nil()),
            Vector3f(0, 0, 0),
            0, // "object-like" point center-bounds
            1,
            "", // no mesh
            true
        );
    }

    // Helper that traverses the query handler and checks that it's structure
    // matches the set of nodes given
    void verifyNodesMatch(const NodeMap& expected) {
        size_t matched_count = 0;
        for(QueryHandler::NodeIterator nit = handler->nodesBegin(); nit != handler->nodesEnd(); nit++) {
            NodeMap::const_iterator expected_it = expected.find(nit.id());
            TS_ASSERT(expected_it != expected.end());
            if (expected_it == expected.end()) continue;

            TS_ASSERT_EQUALS(nit.parentId(), expected_it->second.parent);
            matched_count++;
        }
        TS_ASSERT_EQUALS(matched_count, expected.size());
    }

    // Test that basic refinement works properly
    void testAdditionsRemovals() {
        init(true, true);

        NodeMap expected;
        // Should start out matching empty
        verifyNodesMatch(expected);

        // Add root
        expected[GenerateObjectID(1)] = Node(GenerateObjectID(1), ObjectID::null(), false);
        addReplicatedObject(expected[GenerateObjectID(1)]);
        verifyNodesMatch(expected);

        // Add root children
        expected[GenerateObjectID(2)] = Node(GenerateObjectID(2), GenerateObjectID(1), false);
        addReplicatedObject(expected[GenerateObjectID(2)]);
        verifyNodesMatch(expected);

        expected[GenerateObjectID(3)] = Node(GenerateObjectID(3), GenerateObjectID(1), false);
        addReplicatedObject(expected[GenerateObjectID(3)]);
        verifyNodesMatch(expected);

        expected[GenerateObjectID(4)] = Node(GenerateObjectID(4), GenerateObjectID(1), false);
        addReplicatedObject(expected[GenerateObjectID(4)]);
        verifyNodesMatch(expected);

        // Add root grandchildren, as objects
        expected[GenerateObjectID(5)] = Node(GenerateObjectID(5), GenerateObjectID(2), true);
        addReplicatedObject(expected[GenerateObjectID(5)]);
        verifyNodesMatch(expected);

        expected[GenerateObjectID(6)] = Node(GenerateObjectID(6), GenerateObjectID(2), true);
        addReplicatedObject(expected[GenerateObjectID(6)]);
        verifyNodesMatch(expected);

        expected[GenerateObjectID(7)] = Node(GenerateObjectID(7), GenerateObjectID(3), true);
        addReplicatedObject(expected[GenerateObjectID(7)]);
        verifyNodesMatch(expected);

        expected[GenerateObjectID(8)] = Node(GenerateObjectID(8), GenerateObjectID(3), true);
        addReplicatedObject(expected[GenerateObjectID(8)]);
        verifyNodesMatch(expected);

        // Remove the one empty child of the root
        expected.erase(GenerateObjectID(4));
        loccache->removeObject(GenerateObjectID(4));
        verifyNodesMatch(expected);

        // Remove rest of nodes, bottom up, but in different order from addition.
        expected.erase(GenerateObjectID(6));
        loccache->removeObject(GenerateObjectID(6));
        verifyNodesMatch(expected);

        expected.erase(GenerateObjectID(5));
        loccache->removeObject(GenerateObjectID(5));
        verifyNodesMatch(expected);

        expected.erase(GenerateObjectID(2));
        loccache->removeObject(GenerateObjectID(2));
        verifyNodesMatch(expected);

        expected.erase(GenerateObjectID(8));
        loccache->removeObject(GenerateObjectID(8));
        verifyNodesMatch(expected);

        expected.erase(GenerateObjectID(7));
        loccache->removeObject(GenerateObjectID(7));
        verifyNodesMatch(expected);

        expected.erase(GenerateObjectID(3));
        loccache->removeObject(GenerateObjectID(3));
        verifyNodesMatch(expected);

        expected.erase(GenerateObjectID(1));
        loccache->removeObject(GenerateObjectID(1));
        verifyNodesMatch(expected);
    }
};

#endif //_LIBPROX_TEST_RTREE_REPLICATED_TREE_TEST_HPP_
