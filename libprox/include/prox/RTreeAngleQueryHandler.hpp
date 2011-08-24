/*  libprox
 *  RTreeAngleQueryHandler.hpp
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

#ifndef _PROX_RTREE_ANGLE_QUERY_HANDLER_HPP_
#define _PROX_RTREE_ANGLE_QUERY_HANDLER_HPP_

#include <prox/RTreeQueryHandler.hpp>

namespace Prox {

/** RTreeAngleQueryHandler implements solid angle query processing (return
 *  objects with > solid angle omega, if requested, constrain by distance and
 *  choose the top N).
 */
template<typename SimulationTraits = DefaultSimulationTraits>
class RTreeAngleQueryHandler : public RTreeQueryHandler<SimulationTraits> {
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


    static RTreeAngleQueryHandler* construct(uint16 elements_per_node) {
        return new RTreeAngleQueryHandler(elements_per_node);
    }
    static QueryHandlerCreator Constructor(uint16 elements_per_node) {
        return std::tr1::bind(&RTreeAngleQueryHandler::construct, elements_per_node);
    }


    RTreeAngleQueryHandler(uint16 elements_per_node)
     : RTreeQueryHandler<SimulationTraits>(elements_per_node)
    {
    }

    virtual ~RTreeAngleQueryHandler() {
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
            QueryCacheType newcache(query->maxResults());

            Vector3 qpos = query->position(t);
            BoundingSphere qregion = query->region();
            float qmaxsize = query->maxSize();
            const SolidAngle& qangle = query->angle();
            float qradius = query->radius();

            std::stack<RTreeNodeType*> node_stack;
            node_stack.push(mRTree->root());
            while(!node_stack.empty()) {
                RTreeNodeType* node = node_stack.top();
                node_stack.pop();

                if (node->leaf()) {
                    for(int i = 0; i < node->size(); i++) {
                        tcount++;
                        float32 score = node->childData(i,mLocCache,t).score(qpos, qregion, qmaxsize, qangle, qradius);
                        if (score != -1)
                            newcache.add(mLocCache->iteratorID(node->object(i).object), score);
                        else
                            ncount++;
                    }
                }
                else {
                    for(int i = 0; i < node->size(); i++) {
                        tcount++;
                        bool satisfies = node->childData(i,mLocCache,t).satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
                        if (satisfies)
                            node_stack.push(node->node(i));
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
    using RTreeQueryHandlerType::mLocCache;
    using RTreeQueryHandlerType::mRTree;
    using RTreeQueryHandlerType::mRemovedObjects;
    using RTreeQueryHandlerType::mQueries;
    using RTreeQueryHandlerType::mLastTime;
}; // class RTreeAngleQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_ANGLE_QUERY_HANDLER_HPP_
