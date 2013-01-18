// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_RTREE_COMPLEX_REPLICATION_EVENTS_TEST_HPP_
#define _LIBPROX_TEST_RTREE_COMPLEX_REPLICATION_EVENTS_TEST_HPP_

#include <cxxtest/TestSuite.h>
#include <prox/base/DefaultSimulationTraits.hpp>
#include "TestLocationServiceCache.hpp"
#include <prox/base/QueryEventListener.hpp>
#include <prox/base/AggregateListener.hpp>
#include <prox/manual/RTreeManualQueryHandler.hpp>
#include "TestSettings.hpp"

// Tests focused on complex events that occur and could mess with replication,
// e.g. splits/merges in the orignal RTree. These are isolated because the setup
// and making sure changes to the RTree code doesn't change the way they behave
// (e.g. insertion algorithms) is harder, so only a few tests use this
// setup. Tests that can be performed using an easier to manage setup (e.g. as
// in RTreeTreeReplicatedFromQueryTest) should go there instead.
class RTreeComplexReplicationEventsTest : public CxxTest::TestSuite,
    public Prox::QueryEventListener<Prox::DefaultSimulationTraits, Prox::ManualQuery<Prox::DefaultSimulationTraits> >,
    public Prox::AggregateListener<Prox::DefaultSimulationTraits>
{
public:
    typedef Prox::ManualQueryHandler<Prox::DefaultSimulationTraits> QueryHandler;
    typedef Prox::ManualQuery<Prox::DefaultSimulationTraits> Query;
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

    // This setup is actually just like RTreeTreeReplicatedFromQueryTest
    // structurally, but instead of treating orig_loccache and orig_handler as
    // if they were themselves replicated, we treat them as originals (i.e. we
    // don't pass in parent information). This means that a) getting the tree
    // layout and events we want is harder and b) we have to verify we've got
    // what we've tried to construct.

    TestLocationServiceCache* orig_loccache;
    QueryHandler* orig_handler;
    TestLocationServiceCache* replicated_loccache;
    QueryHandler* replicated_handler;

    // We'll have 2 queries, one in the original tree controlling replication
    Query* orig_query;
    // And one in the replicated tree, to ensure operations on those cuts are
    // handled properly
    Query* replicated_query;

    RTreeComplexReplicationEventsTest()
     : orig_loccache(NULL),
       orig_handler(NULL),
       replicated_loccache(NULL),
       replicated_handler(NULL),
       orig_query(NULL),
       replicated_query(NULL)
    {}

    // AggregateListener Interface -- for query handlers that return aggregates
    // we need to add/remove aggregates to the loccache. Normally these would
    // arrive via the LocationService, but we just push them in directly here to
    // avoid including more pieces.
    virtual void aggregateCreated(AggregatorType* handler, const ObjectIDType& objid) {
        if (handler == orig_handler) {
            orig_loccache->addObject(
                objid,
                true, // aggregate
                MotionVector3f(Time::null(), Vector3f(0, 0, 0), Vector3f::nil()),
                Vector3f(0, 0, 0), 0, 0,
                "", // no mesh
                false
            );
        }
    }
    virtual void aggregateChildAdded(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) {
    }
    virtual void aggregateChildRemoved(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) {
    }
    virtual void aggregateBoundsUpdated(AggregatorType* handler, const ObjectIDType& objid,
        const Vector3Type& bnds_center_offset, const realType bnds_center_bounds_radius, const realType bnds_max_object_size) {
        if (handler == orig_handler)
            orig_loccache->updateBounds(objid, bnds_center_offset, bnds_center_bounds_radius, bnds_max_object_size);
    }
    virtual void aggregateDestroyed(AggregatorType* handler, const ObjectIDType& objid) {
        if (handler == orig_handler)
            orig_loccache->removeObject(objid);
    }
    virtual void aggregateObserved(AggregatorType* handler, const ObjectIDType& objid, uint32 nobservers, uint32 nchildren) {}


    void setUp() {
        // Note small branching factor to make tests smaller when checking split behavior
        orig_loccache = new TestLocationServiceCache();
        orig_handler = new Prox::RTreeManualQueryHandler<Prox::DefaultSimulationTraits, TestNodeData>(3);
        orig_handler->setAggregateListener(this);
        orig_handler->initialize(
            orig_loccache, orig_loccache,
            true, false
        );

        replicated_loccache = new TestLocationServiceCache();
        replicated_handler = new Prox::RTreeManualQueryHandler<Prox::DefaultSimulationTraits, TestNodeData>(3);
        replicated_handler->initialize(
            replicated_loccache, replicated_loccache,
            true, true
        );

        orig_query = orig_handler->registerQuery(
            MotionVector3f(Time::null(), Vector3f::nil(), Vector3f::nil()),
            BoundingSphere3f(Vector3f::nil(), 0.f),
            0
        );
        orig_query->setEventListener(this);

        replicated_query = replicated_handler->registerQuery(
            MotionVector3f(Time::null(), Vector3f::nil(), Vector3f::nil()),
            BoundingSphere3f(Vector3f::nil(), 0.f),
            0
        );
        replicated_query->setEventListener(this);
    }

    void tearDown() {
        assert(replicated_query != NULL);
        delete replicated_query; replicated_query = NULL;
        assert(orig_query != NULL);
        delete orig_query; orig_query = NULL;

        delete orig_handler; orig_handler = NULL;
        delete orig_loccache; orig_loccache = NULL;

        delete replicated_handler; replicated_handler = NULL;
        delete replicated_loccache; replicated_loccache = NULL;
    }

    // Note no parents -- these have to be inserted via the rtree insertion
    // code. Instead, provide positions which will ensure things get grouped
    // together (i.e. identical positions)
    void addObject(const ObjectID& id, const Vector3f& pos) {
        //std::cout << "creating object " << id.toString() << std::endl;
        orig_loccache->addObject(
            id,
            false,
            MotionVector3f(Time::null(), pos, Vector3f::nil()),
            Vector3f(0, 0, 0),
            0, // "object-like" point center-bounds
            1,
            "", // no mesh
            true
        );
    }
    void removeObject(const ObjectID& id) {
        orig_loccache->removeObject(id);
    }

    // Forwards object events from the orignal tree into the replicated location
    // service cache, which puts them into the query handler
    virtual void queryHasEvents(Query* query) {
        // Original query results -> replicated_loccache
        if (query == orig_query) {
            typedef std::deque<QueryEvent> QueryEventList;
            QueryEventList evts;
            query->popEvents(evts);

            while(!evts.empty()) {
                const QueryEvent& evt = evts.front();

                for(size_t aidx = 0; aidx < evt.additions().size(); aidx++) {
                    QueryEvent::Addition add = evt.additions()[aidx];
                    replicated_loccache->addObjectWithParent(
                        add.id(), add.parent(),
                        (add.type() == QueryEvent::Imposter),
                        orig_loccache->location(add.id()),
                        orig_loccache->centerOffset(add.id()),
                        orig_loccache->centerBoundsRadius(add.id()),
                        orig_loccache->maxSize(add.id()),
                        orig_loccache->mesh(add.id()),
                        true
                    );
                }

                for(size_t pidx = 0; pidx < evt.reparents().size(); pidx++) {
                    QueryEvent::Reparent rep = evt.reparents()[pidx];
                    replicated_loccache->updateParent(rep.id(), rep.newParent());
                }

                for(size_t ridx = 0; ridx < evt.removals().size(); ridx++) {
                    QueryEvent::Removal rem = evt.removals()[ridx];
                    replicated_loccache->removeObject(rem.id());
                }

                evts.pop_front();
            }
        }
    }

    // Helper that verifies the two query handler trees have identical
    // structure. Note that we don't check that they are fully identical, just
    // that the replicated data matches the original structure.
    void verifyTreesMatch() {
        // The trees may not be replicated exactly, we really just
        // need to make sure they share the same structure and we want
        // to make sure we check it by traversing the actual data
        // structure (as opposed to, e.g., getting it from the
        // location service cache).
        typedef std::map<ObjectID, ObjectID> ParentMap;
        ParentMap orig_pm, replicated_pm;
        for(QueryHandler::NodeIterator orig_it = orig_handler->nodesBegin();
            orig_it != orig_handler->nodesEnd();
            orig_it++)
        {
            orig_pm.insert(std::make_pair(orig_it.id(), orig_it.parentId()));
        }
        for(QueryHandler::NodeIterator replicated_it = replicated_handler->nodesBegin();
            replicated_it != replicated_handler->nodesEnd();
            replicated_it++)
        {
            replicated_pm.insert(std::make_pair(replicated_it.id(), replicated_it.parentId()));
        }

        // Then check that they really match
        for(ParentMap::const_iterator replicated_it = replicated_pm.begin();
            replicated_it != replicated_pm.end();
            replicated_it++)
        {
            ParentMap::const_iterator orig_it = orig_pm.find(replicated_it->first);
            TS_ASSERT_DIFFERS(orig_it, orig_pm.end());
            if (orig_it == orig_pm.end()) {
                std::cout << "Didn't find " << replicated_it->first << " in original tree but it was in the replicated tree." << std::endl;
                continue;
            }
            TS_ASSERT_EQUALS(orig_it->second, replicated_it->second);
        }
    }
// Provide this to check that trees are truly identical, rather than just that
// the replicated data matches
#define TS_ASSERT_TREES_IDENTICAL()                                     \
        TS_ASSERT_EQUALS(orig_handler->numNodes(), replicated_handler->numNodes()); \
        TS_ASSERT_EQUALS(orig_handler->numObjects(), replicated_handler->numObjects()); \
        verifyTreesMatch();

    static ObjectID ObjID(size_t id) {
        char raw[ObjectID::static_size];
        memset(raw, 0, ObjectID::static_size);
        ((size_t*)raw)[0] = id;
        return ObjectID(raw, ObjectID::static_size);
    }

    bool refineNode(Query* query, const ObjectID& objid) {
        bool res = query->refine(objid);
        return res;
    }

    // Force refinement to the bottom of the current tree, assumes the
    // cut is at the root
    void refineToBottom(Query* query) {
        for(QueryHandler::NodeIterator nit = query->handler()->nodesBegin(); nit != query->handler()->nodesEnd(); nit++) {
            // It would be nice to assert success here, but we can't
            // because when we reach replicated leaf nodes the call
            // will fail
            refineNode(query, nit.id());
        }
    }

    // Empty replicated trees should be the same
    void testEmptyTree() {
        TS_ASSERT_TREES_IDENTICAL();
    }

#define TS_ASSERT_SAME_PARENTS(ii, jj)                                  \
    TS_ASSERT_EQUALS(replicated_loccache->parent(ObjID(ii)), replicated_loccache->parent(ObjID(jj)))
#define TS_ASSERT_PARENT(ii, jj)                                        \
    TS_ASSERT_EQUALS(replicated_loccache->parent(ObjID(ii)), ObjID(jj))

    // Sanity check our ability to build trees we want
    void testBuildTree() {
// Only perform these tests if we're not lifting cuts. With cut
// lifting, these are uninteresting since cuts will be lifted above
// them anyway. However, we need to do this for the body of each test because
// cxxtest can't handle preprocessor macros
#ifndef LIBPROX_LIFT_CUTS
        // Insert 4 nodes, which should cause a split node and force
        // them to be grouped
        addObject(ObjID(1), Vector3f(0, 0, 0));
        addObject(ObjID(2), Vector3f(0, 0, 0));
        addObject(ObjID(3), Vector3f(1, 0, 0));
        addObject(ObjID(4), Vector3f(1, 0, 0));
        refineToBottom(orig_query);

        TS_ASSERT_SAME_PARENTS(1, 2);
        TS_ASSERT_SAME_PARENTS(3, 4);

        // Make sure additional nodes go to the correct parents
        addObject(ObjID(5), Vector3f(0, 0, 0));
        addObject(ObjID(6), Vector3f(1, 0, 0));
        refineToBottom(orig_query);

        TS_ASSERT_SAME_PARENTS(5, 1);
        TS_ASSERT_SAME_PARENTS(6, 3);
#endif //ndef LIBPROX_LIFT_CUTS
    }

    // Test that a cascading split where nodes above the cut split
    void testCascadeSplitAboveCut() {
#ifndef LIBPROX_LIFT_CUTS
        // NOTE that this test kind of depends on tree construction,
        // so in a sense it could break. The comments about where
        // cascading splits happen could be wrong, but we should
        // definitely be adding enough to trigger them to happen
        // *somewhere* before we make the checks.

        // Build up a tree that will cause a cascading split. After
        // this we'll have root -> child -> 3 children -> objects.
        addObject(ObjID(1), Vector3f(0, 0, 0));
        addObject(ObjID(2), Vector3f(0, 0, 0));
        addObject(ObjID(3), Vector3f(1, 0, 0));
        addObject(ObjID(4), Vector3f(1, 0, 0));
        addObject(ObjID(5), Vector3f(0, 0, 0));

        addObject(ObjID(7), Vector3f(0, 0, 0));
        addObject(ObjID(9), Vector3f(0, 0, 0));
        // This step actually creates a cascading split first, from
        // root -> 3 children -> 3 objects, where the additional
        // object causes splits. We haven't refined the cut yet, so
        // this happens below the cut.
        addObject(ObjID(10), Vector3f(0, 0, 0));

        addObject(ObjID(11), Vector3f(0, 0, 0));
        refineToBottom(orig_query);
        TS_ASSERT_TREES_IDENTICAL();
        // NOTE that these numbers are implementation specific, so if
        // they start failing it doesn't mean anything is necessarily
        // wrong (e.g. you could get here and have only 16 nodes, but
        // still have all objects, have caused a cascading split, and
        // still have cuts in good shape), just that you need to make
        // sure we're still testing what we think we are and update
        // the values.
        TS_ASSERT_EQUALS(replicated_handler->numNodes(), 16);

        // Adding this object should overflow one of the 3 children
        // nodes mentioned above, causing a split, and the child of
        // the root to also split as it will end up with 4 children
        addObject(ObjID(12), Vector3f(0, 0, 0));

        TS_ASSERT_TREES_IDENTICAL();
        // Without cut lifting, we should end up at the same
        // level. We'll have also added 1 new object and split 2
        // nodes, adding 3 to our total count of nodes
        TS_ASSERT_EQUALS(replicated_handler->numNodes(), 19);
#endif //ndef LIBPROX_LIFT_CUTS
    }

    // Test similar to above where a split occurs above the cut, but
    // in this version, a node above the cut is reparented as part of
    // the splitting process (which requires deeper tree -- the
    // version above has reparenting along the cut only).
    void testCascadeSplitAboveCutReparentingAboveCut() {
#ifndef LIBPROX_LIFT_CUTS
        // See note about tree construction above.

        // For this version, we need something big enough that the cascading
        // split causing reparenting above the cut. This means we have to have
        // the cut below a child node that is being moved, i.e. the grandparent
        // or higher needs to be getting split. This effectively means we need
        // to have a cascading split of at least 3 nodes.
        addObject(ObjID(1), Vector3f(0, 0, 0));
        addObject(ObjID(2), Vector3f(0, 0, 0));
        addObject(ObjID(3), Vector3f(1, 0, 0));
        addObject(ObjID(4), Vector3f(1, 0, 0));
        addObject(ObjID(5), Vector3f(0, 0, 0));
        addObject(ObjID(6), Vector3f(0, 0, 0));
        addObject(ObjID(7), Vector3f(0, 0, 0));
        addObject(ObjID(8), Vector3f(0, 0, 0));
        addObject(ObjID(9), Vector3f(0, 0, 0));
        addObject(ObjID(10), Vector3f(0, 0, 0));
        addObject(ObjID(11), Vector3f(0, 0, 0));
        addObject(ObjID(12), Vector3f(0, 0, 0));
        addObject(ObjID(13), Vector3f(0, 0, 0));

        // Based on the current build setup, this next addition should cause a
        // split. A few sanity checks to make sure the tree looks how we think
        // it does:

        // Then make sure we've got the query down at the bottom of the tree
        refineToBottom(orig_query);
        TS_ASSERT_TREES_IDENTICAL();
        // These are the nodes that should be grouped together and the new node
        // being added to them will trigger a cascading split (all the way to
        // the root).
        TS_ASSERT_SAME_PARENTS(1, 12);
        TS_ASSERT_SAME_PARENTS(1, 13);
        TS_ASSERT_EQUALS(orig_handler->numNodes(), 23);

        addObject(ObjID(14), Vector3f(0, 0, 0));

#endif //ndef LIBPROX_LIFT_CUTS
    }
};

#endif //_LIBPROX_TEST_RTREE_COMPLEX_REPLICATION_EVENTS_TEST_HPP_
