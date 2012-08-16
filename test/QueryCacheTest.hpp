// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include <cxxtest/TestSuite.h>
#include <prox/base/QueryCache.hpp>
#include <prox/base/DefaultSimulationTraits.hpp>
#include <prox/base/LocationServiceCache.hpp>

class QueryCacheTest : public CxxTest::TestSuite {
public:
    typedef Prox::QueryCache<Prox::DefaultSimulationTraits> QueryCache;
    typedef QueryCache::ObjectID ObjectID;
    typedef QueryCache::ObjectIDSet ObjectIDSet;
    typedef Prox::QueryEvent<Prox::DefaultSimulationTraits> QueryEvent;

    typedef std::vector<ObjectID> ObjectList;
    ObjectList objects;

    // Location service implementation that doesn't actually do anything, only
    // required since QueryEvent makes calls to it.
    class MockLocationServiceCache : public Prox::LocationServiceCache<Prox::DefaultSimulationTraits> {
    public:
        virtual ~MockLocationServiceCache() {}

        virtual void addUpdateListener(LocationUpdateListenerType* listener) {}
        virtual void removeUpdateListener(LocationUpdateListenerType* listener) {}

        virtual void addPlaceholderImposter(
            const ObjectID& id,
            const Vector3& center_pos,
            const Prox::float32 center_bounds_radius,
            const Prox::float32 max_size,
            const Prox::String& zernike,
            const Prox::String& mesh
        ) {}
        virtual Iterator startTracking(const ObjectID& id) { return Iterator(); }
        virtual void stopTracking(const Iterator& id) {}
        virtual bool startRefcountTracking(const ObjectID& id) { return true; }
        virtual void stopRefcountTracking(const ObjectID& id) { }
        virtual MotionVector3 location(const Iterator& id) { return MotionVector3(Time::null(), Vector3::nil(), Vector3::nil()); }
        virtual Vector3 centerOffset(const Iterator& id) { return Vector3(0, 0, 0); }
        virtual Prox::float32 centerBoundsRadius(const Iterator& id) { return 0.f; }
        virtual Prox::float32 maxSize(const Iterator& id) { return 0.f; }
        virtual Prox::ZernikeDescriptor& zernikeDescriptor(const Iterator& id) { }
        virtual Prox::String mesh(const Iterator& id) { return ""; }


        virtual bool isLocal(const Iterator& id) { return true; }
        virtual const ObjectID& iteratorID(const Iterator& id) { }
    };

    void setUp() {
        for(size_t i = 0; i < 100; i++)
            objects.push_back(ObjectID::Random()());
    }

    void testAdd() {
        QueryCache qc(1);
        qc.add(objects[0], 0);
        TS_ASSERT_EQUALS(qc.size(), 1);
        TS_ASSERT(qc.contains(objects[0]));
    }

    void testRemove() {
        QueryCache qc(1);
        qc.add(objects[0], 0);
        qc.remove(objects[0]);
        TS_ASSERT_EQUALS(qc.size(), 0);
        TS_ASSERT(!qc.contains(objects[0]));
    }

    void testOverflowSmall() {
        QueryCache qc(1);
        qc.add(objects[0], 1);
        qc.add(objects[1], 2);
        TS_ASSERT_EQUALS(qc.size(), 1);
        TS_ASSERT(!qc.contains(objects[0]));
        TS_ASSERT(qc.contains(objects[1]));
    }

    void testOverflowLarge() {
        QueryCache qc(10);
        // Last 10 sould stay in the cache because they have higher scores
        for(size_t i = 0; i < 20; i++)
            qc.add(objects[i], i);
        TS_ASSERT_EQUALS(qc.size(), 10);
        for(size_t i = 0; i < 20; i++) {
            if (i < 10) {
                TS_ASSERT(!qc.contains(objects[i]));
            }
            else {
                TS_ASSERT(qc.contains(objects[i]));
            }
        }
    }

    void testFull() {
        QueryCache qc(10);
        for(size_t i = 0; i < 20; i++) {
            qc.add(objects[i], i);
            if (i < 9) {
                TS_ASSERT(!qc.full());
            }
            else {
                TS_ASSERT(qc.full());
            }
        }
    }

    void testExchangeAdditions() {
        // Tests exchange() when the new set has a superset of the old one (only
        // additions)
        QueryCache qc1(10);
        QueryCache qc2(10);

        for(size_t i = 0; i < 10; i++) {
            if (i < 5) qc1.add(objects[i], i);
            qc2.add(objects[i], i);
        }

        Prox::QueryHandlerIndexID qhiid = 47;
        MockLocationServiceCache loccache;
        std::deque<QueryEvent> changes;
        ObjectIDSet permanent_removals;
        qc1.exchange(qhiid, &loccache, qc2, &changes, permanent_removals);

        // Check correct sizes
        TS_ASSERT_EQUALS(qc1.size(), 10);
        TS_ASSERT_EQUALS(changes.size(), 5);
        // That we have the members we expect
        for(size_t i = 0; i < 10; i++) {
            TS_ASSERT(qc1.contains(objects[i]));
        }
        // And that we've generated appropriate events
        while(!changes.empty()) {
            TS_ASSERT_EQUALS(changes.front().indexID(), qhiid);
            TS_ASSERT_EQUALS(changes.front().size(), 1);
            TS_ASSERT_EQUALS(changes.front().additions().size(), 1);
            TS_ASSERT_EQUALS(changes.front().removals().size(), 0);
            ObjectList::iterator added_objects_begin = objects.begin()+5, added_objects_end = objects.begin()+10;
            TS_ASSERT_DIFFERS(
                std::find(added_objects_begin, added_objects_end, changes.front().additions().front().id()),
                added_objects_end
            );
            changes.pop_front();
        }
    }

    void testExchangeRemovals() {
        // Tests exchange() when the old set has a superset of the new one (only
        // removals)
        QueryCache qc1(10);
        QueryCache qc2(10);

        for(size_t i = 0; i < 10; i++) {
            qc1.add(objects[i], i);
            if (i < 5) qc2.add(objects[i], i);
        }

        Prox::QueryHandlerIndexID qhiid = 47;
        MockLocationServiceCache loccache;
        std::deque<QueryEvent> changes;
        ObjectIDSet permanent_removals;
        qc1.exchange(qhiid, &loccache, qc2, &changes, permanent_removals);

        // Check correct sizes
        TS_ASSERT_EQUALS(qc1.size(), 5);
        TS_ASSERT_EQUALS(changes.size(), 5);
        // That we have the members we expect
        for(size_t i = 0; i < 5; i++) {
            TS_ASSERT(qc1.contains(objects[i]));
        }
        for(size_t i = 5; i < 10; i++) {
            TS_ASSERT(!qc1.contains(objects[i]));
        }
        // And that we've generated appropriate events
        while(!changes.empty()) {
            TS_ASSERT_EQUALS(changes.front().indexID(), qhiid);
            TS_ASSERT_EQUALS(changes.front().size(), 1);
            TS_ASSERT_EQUALS(changes.front().additions().size(), 0);
            TS_ASSERT_EQUALS(changes.front().removals().size(), 1);
            ObjectList::iterator removed_objects_begin = objects.begin()+5, removed_objects_end = objects.begin()+10;
            TS_ASSERT_DIFFERS(
                std::find(removed_objects_begin, removed_objects_end, changes.front().removals().front().id()),
                removed_objects_end
            );
            changes.pop_front();
        }
    }

    void testExchangeAdditionsAndRemovals() {
        // Tests exchange() when the there are differences in both sets,
        // i.e. additions and removals
        // Tests exchange() when the new set has a superset of the old one (only
        // additions)
        QueryCache qc1(10);
        QueryCache qc2(10);

        ObjectList added_objects;
        ObjectList removed_objects;
        for(size_t i = 0; i < 10; i++) {
            // 3 types. stays the same:
            if (i < 3) {
                qc1.add(objects[i], i);
                qc2.add(objects[i], i);
            }
            else if (i < 6) { // added
                qc2.add(objects[i], i);
                added_objects.push_back(objects[i]);
            }
            else { // removed
                qc1.add(objects[i], i);
                removed_objects.push_back(objects[i]);
            }
        }

        Prox::QueryHandlerIndexID qhiid = 47;
        MockLocationServiceCache loccache;
        std::deque<QueryEvent> changes;
        ObjectIDSet permanent_removals;
        qc1.exchange(qhiid, &loccache, qc2, &changes, permanent_removals);

        // Check correct sizes
        TS_ASSERT_EQUALS(qc1.size(), 6);
        TS_ASSERT_EQUALS(changes.size(), 7);
        // That we have the members we expect
        for(size_t i = 0; i < 6; i++) {
            TS_ASSERT(qc1.contains(objects[i]));
        }
        for(size_t i = 6; i < 10; i++) {
            TS_ASSERT(!qc1.contains(objects[i]));
        }
        // And that we've generated appropriate events
        int nadditions = 0, nremovals = 0;
        while(!changes.empty()) {
            TS_ASSERT_EQUALS(changes.front().indexID(), qhiid);
            TS_ASSERT_EQUALS(changes.front().size(), 1);
            if (!changes.front().additions().empty()) {
                nadditions++;
                TS_ASSERT_EQUALS(changes.front().additions().size(), 1);
                TS_ASSERT_EQUALS(changes.front().removals().size(), 0);
                TS_ASSERT_DIFFERS(
                    std::find(added_objects.begin(), added_objects.end(), changes.front().additions().front().id()),
                    added_objects.end()
                );
            }
            else {
                TS_ASSERT(!changes.front().removals().empty());
                nremovals++;
                TS_ASSERT_EQUALS(changes.front().additions().size(), 0);
                TS_ASSERT_EQUALS(changes.front().removals().size(), 1);
                TS_ASSERT_DIFFERS(
                    std::find(removed_objects.begin(), removed_objects.end(), changes.front().removals().front().id()),
                    removed_objects.end()
                );
            }
            changes.pop_front();
        }
        TS_ASSERT_EQUALS(nadditions, 3);
        TS_ASSERT_EQUALS(nremovals, 4);
    }
};
