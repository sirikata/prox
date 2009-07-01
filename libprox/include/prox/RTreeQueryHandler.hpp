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
#include <prox/BoundingBox.hpp>

namespace Prox {

class BoundingSphereData;
class MaxSphereData;

template<typename NodeData>
struct RTreeNode;

class RTreeQueryHandler : public QueryHandler, public LocationUpdateListener, public QueryChangeListener {
public:
    RTreeQueryHandler(uint8 elements_per_node);
    virtual ~RTreeQueryHandler();

    virtual void initialize(LocationServiceCache* loc_cache);
    virtual void registerObject(const ObjectID& obj);
    virtual void registerQuery(Query* query);
    virtual void tick(const Time& t);

    // LocationUpdateListener Implementation
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3f& old_pos, const MotionVector3f& new_pos);
    virtual void locationBoundsUpdated(const ObjectID& obj_id, const BoundingSphere3f& old_bounds, const BoundingSphere3f& new_bounds);
    virtual void locationDisconnected(const ObjectID& obj_id);

    // QueryChangeListener Implementation
    virtual void queryPositionUpdated(Query* query, const MotionVector3f& old_pos, const MotionVector3f& new_pos);
    virtual void queryDeleted(const Query* query);

private:
    void insert(const ObjectID& obj_id, const Time& t);
    void deleteObj(const ObjectID& obj_id, const Time& t);

    struct QueryState {
        QueryCache cache;
    };

    typedef std::set<ObjectID> ObjectSet;
    typedef std::map<Query*, QueryState*> QueryMap;

    //typedef RTreeNode<BoundingSphereData> RTree;
    typedef RTreeNode<MaxSphereData> RTree;

    LocationServiceCache* mLocCache;

    RTree* mRTreeRoot;
    ObjectSet mObjects;
    QueryMap mQueries;
    Time mLastTime;
}; // class RTreeQueryHandler

} // namespace Prox

#endif //_PROX_RTREE_QUERY_HANDLER_HPP_
