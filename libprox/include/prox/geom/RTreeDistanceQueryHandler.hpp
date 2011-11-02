/*  libprox
 *  RTreeDistanceQueryHandler.hpp
 *
 *  Copyright (c) 2011, Ewen Cheslack-Postava
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

#ifndef _PROX_RTREE_DISTANCE_QUERY_HANDLER_HPP_
#define _PROX_RTREE_DISTANCE_QUERY_HANDLER_HPP_

#include <prox/geom/RTreeQueryHandler.hpp>

namespace Prox {

/** RTreeDistanceQueryHandler implements distance query processing (return
 *  objects closer than distance d, and if requested, choose the top N).
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class RTreeDistanceQueryHandler : public RTreeQueryHandler<SimulationTraits> {
public:
    typedef RTreeQueryHandler<SimulationTraits> RTreeQueryHandlerType;

    typedef typename RTreeQueryHandlerType::QueryHandlerType QueryHandlerType;

    typedef typename RTreeQueryHandlerType::Time Time;
    typedef typename RTreeQueryHandlerType::Vector3 Vector3;
    typedef typename RTreeQueryHandlerType::BoundingSphere BoundingSphere;
    typedef typename RTreeQueryHandlerType::SolidAngle SolidAngle;

    typedef typename RTreeQueryHandlerType::QueryCacheType QueryCacheType;
    typedef typename RTreeQueryHandlerType::QueryMap QueryMap;
    typedef typename RTreeQueryHandlerType::QueryMapIterator QueryMapIterator;
    typedef typename RTreeQueryHandlerType::QueryType QueryType;
    typedef typename RTreeQueryHandlerType::QueryEventType QueryEventType;
    typedef typename RTreeQueryHandlerType::QueryState QueryState;

    typedef typename RTreeQueryHandlerType::QueryHandlerCreator QueryHandlerCreator;


    typedef typename RTreeQueryHandlerType::RTreeNodeType RTreeNodeType;


    static RTreeDistanceQueryHandler* construct(uint16 elements_per_node) {
        return new RTreeDistanceQueryHandler(elements_per_node);
    }
    static QueryHandlerCreator Constructor(uint16 elements_per_node) {
        return std::tr1::bind(&RTreeDistanceQueryHandler::construct, elements_per_node);
    }


    RTreeDistanceQueryHandler(uint16 elements_per_node)
     : RTreeQueryHandler<SimulationTraits>(elements_per_node)
    {
    }

    virtual ~RTreeDistanceQueryHandler() {
    }

    void tick(const Time& t, bool report) {
        RTreeQueryHandlerType::preTick(t, report);

        uint32 nrtnodes = QueryHandlerType::mTrackChecks ? mRTree->size() : 0;

        for(QueryMapIterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
            int tcount = 0; // total
            int ncount = 0; // negatives, anywhere
            int internal_ncount = 0; // negatives, internal

            QueryType* query = query_it->first;
            QueryState* state = query_it->second;

            uint32 qmaxresults = query->maxResults();
            QueryCacheType newcache(qmaxresults);

            Vector3 qpos = query->position(t);
            BoundingSphere qregion = query->region();
            float qmaxsize = query->maxSize();
            float qradius = query->radius();

            // Our basic approach is a standard traversal of the tree. We track
            // a set of outstanding nodes which we've reached but haven't
            // processed, starting with the root. We're going to track them in a
            // max heap (if we're capping the result set size, otherwise we'll
            // use it as a stack to save some effort in maintaining the heap).
            NodeHeap node_heap;
            node_heap.push_back(
                NodeHeapElement(mRTree->root(), scoreDistance(mRTree->root()->data(), qpos, qregion, qmaxsize, qradius))
            );

            bool capped = (qmaxresults != SimulationTraits::InfiniteResults);
            // We can only terminate in one of two ways. If we have no cap on
            // max results, then we keep processing until no unprocessed nodes
            // remain -- the children of a node which satisfied the query may
            // satisfy the query. If there is a cap, then we can stop when the
            // QueryCache is full & there is no chance of improving the set,
            // i.e. the maximum score of the unprocessed nodes is worse than the
            // worst score in the current result set.
            while(!node_heap.empty() && // nodes are left to process
                (!capped || // no cap on number of results
                     // or capped and  not full or possibly better score
                    (!newcache.full() || node_heap.front().score > newcache.minScore() )
                )
            ) {
                // Throughout, we'll have to check whether the result set is
                // capped or not and process push/pop operations as either heap
                // or stack operations. For heaps, we need to do heap push/pop
                // then push/pop back. For stack, just the push/pop back.
                if (capped) std::pop_heap(node_heap.begin(), node_heap.end());
                NodeHeapElement elem = node_heap.back();
                RTreeNodeType* node = elem.node;
                node_heap.pop_back();

                // Process the children of this node. Leaves -> directly to
                // result set. Internal nodes -> inserted into node_heap. Either
                // way, this is where we actually cull.
                if (node->leaf()) {
                    for(int i = 0; i < node->size(); i++) {
                        tcount++;
                        float32 score = scoreDistance( node->childData(i,mLocCache,t), qpos, qregion, qmaxsize, qradius );
                        if (score != -1)
                            newcache.add(mLocCache->iteratorID(node->object(i).object), score);
                        else
                            ncount++;
                    }
                }
                else {
                    for(int i = 0; i < node->size(); i++) {
                        tcount++;
                        float32 score = scoreDistance( node->childData(i,mLocCache,t), qpos, qregion, qmaxsize, qradius );
                        if (score != -1) {
                            node_heap.push_back(NodeHeapElement(node->node(i), score));
                            if (capped) std::push_heap(node_heap.begin(), node_heap.end());
                        }
                        else {
                            internal_ncount++;
                            ncount++;
                        }
                    }
                }
            }

            std::deque<QueryEventType> events;
            state->cache.exchange(newcache, &events, mRemovedObjects);

            query->pushEvents(events);

            if (QueryHandlerType::mTrackChecks && report)
                printf("{ \"id\" : %d, \"nodes\" : %d, \"checks\" : { \"positive\" : %d, \"negative\" : %d, \"negativeinternal\" : %d, \"total\" : %d } }\n", query->id(), nrtnodes, tcount - ncount, ncount, internal_ncount, tcount);

            if (QueryHandlerType::mReportQueryStats && report)
                printf("{ \"id\" : %d, \"checks\" : %d, \"results\" : %d }\n", query->id(), tcount, state->cache.size());
        }
        // We can clear out permanently removed objects since we should have
        // processed all their updates already
        mRemovedObjects.clear();

        RTreeQueryHandlerType::postTick(t, report);
    }

protected:
    struct NodeHeapElement {
        NodeHeapElement(RTreeNodeType* n, float32 s)
         : node(n), score(s)
        {}

        RTreeNodeType* node;
        float32 score;

        bool operator<(const NodeHeapElement& rhs) {
            return score < rhs.score;
        }
    };
    typedef std::vector<NodeHeapElement> NodeHeap;

    typedef typename RTreeQueryHandlerType::NodeData NodeData;

    static float32 scoreDistance(const NodeData& data, const Vector3& qpos, const BoundingSphere& qbounds, const float qmaxsize, const float qradius) {
        // Combine info to reduce amount we need to deal with
        Vector3 query_pos = qpos + qbounds.center();
        // Just sum these since we don't take the solid angle of these, so it is
        // equivalent to extending the radius by the size of the largest object.
        float query_rad = qbounds.radius() + qmaxsize;

        // In this case we only need to know the 'aggregate approximate bounds'
        // for the object, i.e. we don't need to differentiate obj_radius and
        // obj_max_size. Instead, we can use NodeData.getBounds() which sums the
        // two together since these distance checks never use them separately.
        BoundingSphere obj_bounds = data.getBounds();

        // First, a special case is if the query is actually inside the hierarchical bounding sphere, in which
        // case the above description isn't accurate: in this case the worst position for the stand in object
        // is the exact location of the query.  So just let it pass.
        // FIXME we do this check manually for now, but BoundingSphere should provide it
        if (query_rad + obj_bounds.radius() >= (query_pos-obj_bounds.center()).length()) {
            // We give the maximum score here since there's nothing better to
            // give it compared to something that just bumps up against the
            // query. See note below for why it is what it is.
            return qradius*qradius; // - dist_to_obj2, where dist_to_obj2 == 0.
        }

        Vector3 to_obj = obj_bounds.center() - query_pos;
        to_obj = to_obj - to_obj.normal() * (obj_bounds.radius() + query_rad);

        // Must satisfy radius constraint
        assert(qradius != SimulationTraits::InfiniteRadius);
        float dist_to_obj2 = to_obj.lengthSquared();
        if (dist_to_obj2 > qradius*qradius)
            return -1;

        // We want closer to be higher score. dist_to_obj2 is in [0, qradius^2],
        // so just subtract it from qradius^2 to reverse the ordering (closer is
        // better) and ensure the values are still > 0.
        return qradius*qradius - dist_to_obj2;
    }

    using RTreeQueryHandlerType::mLocCache;
    using RTreeQueryHandlerType::mRTree;
    using RTreeQueryHandlerType::mRemovedObjects;
    using RTreeQueryHandlerType::mQueries;
    using RTreeQueryHandlerType::mLastTime;
}; // class RTreeDistanceQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_DISTANCE_QUERY_HANDLER_HPP_
