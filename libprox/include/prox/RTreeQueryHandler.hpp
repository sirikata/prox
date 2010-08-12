/*  libprox
 *  RTreeQueryHandler.hpp
 *
 *  Copyright (c) 2009, Ewen Cheslack-Postava
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of libprox nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PROX_RTREE_QUERY_HANDLER_HPP_
#define _PROX_RTREE_QUERY_HANDLER_HPP_

#include <prox/QueryHandler.hpp>
#include <prox/LocationUpdateListener.hpp>
#include <prox/QueryChangeListener.hpp>
#include <prox/QueryCache.hpp>

#include <prox/DefaultSimulationTraits.hpp>

#include <prox/RTree.hpp>

namespace Prox {

template<typename SimulationTraits = DefaultSimulationTraits>
class RTreeQueryHandler : public QueryHandler<SimulationTraits> {
public:
    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;
    typedef QueryChangeListener<SimulationTraits> QueryChangeListenerType;

    typedef Query<SimulationTraits> QueryType;
    typedef QueryEvent<SimulationTraits> QueryEventType;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef QueryCache<SimulationTraits> QueryCacheType;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;


    RTreeQueryHandler(uint8 elements_per_node)
     : QueryHandlerType(),
       mLocCache(NULL),
       mRTree(NULL),
       mLastTime(Time::null()),
       mElementsPerNode(elements_per_node)
    {
    }

    virtual ~RTreeQueryHandler() {
        for(ObjectSetIterator it = mObjects.begin(); it != mObjects.end(); it++) {
            mLocCache->stopTracking(it->second);
        }
        mObjects.clear();
        for(QueryMapIterator it = mQueries.begin(); it != mQueries.end(); it++) {
            QueryState* state = it->second;
            delete state;
        }
        mQueries.clear();

        mLocCache->removeUpdateListener(this);
    }

    void initialize(LocationServiceCacheType* loc_cache) {
        mLocCache = loc_cache;
        mLocCache->addUpdateListener(this);

        mRTree = new RTree(mElementsPerNode, mLocCache);
    }

    void tick(const Time& t) {
        mRTree->update(t);

        mRTree->verifyConstraints(t);
        int count = 0;
        int ncount = 0;
        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            QueryType* query = query_it->first;
            QueryState* state = query_it->second;
            QueryCacheType newcache;

            Vector3 qpos = query->position(t);
            BoundingSphere qbounds = query->bounds();
            float qradius = query->radius();
            const SolidAngle& qangle = query->angle();

            std::stack<RTreeNodeType*> node_stack;
            node_stack.push(mRTree->root());
            while(!node_stack.empty()) {
                RTreeNodeType* node = node_stack.top();
                node_stack.pop();

                if (node->leaf()) {
                    for(int i = 0; i < node->size(); i++) {
                        count++;
                        if (node->childData(i,mLocCache,t).satisfiesConstraints(qpos, qbounds, qradius, qangle))
                            newcache.add(mLocCache->iteratorID(node->object(i)));
                    }
                }
                else {
                    for(int i = 0; i < node->size(); i++) {
                        count++;
                        if (node->childData(i,mLocCache,t).satisfiesConstraints(qpos, qbounds, qradius, qangle))
                            node_stack.push(node->node(i));
                        else
                            ncount++;
                    }
                }
            }

            std::deque<QueryEventType> events;
            state->cache.exchange(newcache, &events);

            query->pushEvents(events);
        }
        printf("count: %d %d\n", count, ncount);
        mLastTime = t;
    }

    virtual uint32 numObjects() const {
        return (uint32)mObjects.size();
    }
    virtual uint32 numQueries() const {
        return (uint32)mQueries.size();
    }

    void locationConnected(const ObjectID& obj_id, const MotionVector3& pos, const BoundingSphere& bounds) {
        assert(mObjects.find(obj_id) == mObjects.end());
        mObjects[obj_id] = mLocCache->startTracking(obj_id);
        insertObj(obj_id, mLastTime);
    }

    // LocationUpdateListener Implementation
    void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationBoundsUpdated(const ObjectID& obj_id, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
        updateObj(obj_id, mLastTime); // FIXME new time?
    }

    void locationDisconnected(const ObjectID& obj_id) {
        assert( mObjects.find(obj_id) != mObjects.end() );
        LocCacheIterator obj_loc_it = mObjects[obj_id];
        mObjects.erase(obj_id);
        deleteObj(obj_id, mLastTime);
        mLocCache->stopTracking(obj_loc_it);
    }

    // QueryChangeListener Implementation
    void queryPositionChanged(QueryType* query, const MotionVector3& old_pos, const MotionVector3& new_pos) {
        // Nothing to be done, we use values directly from the query
    }

    void queryBoundsChanged(QueryType* query, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
        // XXX FIXME
    }

    void queryAngleChanged(QueryType* query, const SolidAngle& old_val, const SolidAngle& new_val) {
        // XXX FIXME
    }

    void queryDeleted(const QueryType* query) {
        QueryMapIterator it = mQueries.find(const_cast<QueryType*>(query));
        assert( it != mQueries.end() );
        QueryState* state = it->second;
        delete state;
        mQueries.erase(it);
    }

protected:
    void registerQuery(QueryType* query) {
        QueryState* state = new QueryState;
        mQueries[query] = state;
        query->addChangeListener(this);
    }

private:
    void insertObj(const ObjectID& obj_id, const Time& t) {
        mRTree->insert(mObjects[obj_id], t);
    }

    void updateObj(const ObjectID& obj_id, const Time& t) {
        mRTree->update(mObjects[obj_id], t);
    }

    void deleteObj(const ObjectID& obj_id, const Time& t) {
        mRTree->erase(mObjects[obj_id], t);
    }

    struct QueryState {
        QueryCacheType cache;
    };

    typedef std::tr1::unordered_map<ObjectID, LocCacheIterator, typename ObjectID::Hasher> ObjectSet;
    typedef typename ObjectSet::iterator ObjectSetIterator;
    typedef std::tr1::unordered_map<QueryType*, QueryState*> QueryMap;
    typedef typename QueryMap::iterator QueryMapIterator;

    //typedef RTree<SimulationTraits, BoundingSphereData<SimulationTraits> > RTree;
    typedef Prox::RTree<SimulationTraits, MaxSphereData<SimulationTraits> > RTree;
    typedef typename RTree::RTreeNodeType RTreeNodeType;

    LocationServiceCacheType* mLocCache;

    RTree* mRTree;
    ObjectSet mObjects;
    QueryMap mQueries;
    Time mLastTime;
    uint8 mElementsPerNode;
}; // class RTreeQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_QUERY_HANDLER_HPP_
