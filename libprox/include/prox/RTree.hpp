/*  libprox
 *  RTree.hpp
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

#ifndef _PROX_RTREE_HPP_
#define _PROX_RTREE_HPP_

#include <prox/Platform.hpp>
#include <prox/LocationServiceCache.hpp>
#include <prox/Constraints.hpp>
#include <prox/AggregateListener.hpp>
#include <float.h>

#include "RTreeCore.hpp"
#include "RTreeRestructure.hpp"
#include "RTreeBulk.hpp"

namespace Prox {

/**************************************************************************
 * RTree - Wrapper for an RTreeNode root node to provide a nicer interface
 **************************************************************************/
template<typename SimulationTraits, typename NodeData, typename CutNode>
class RTree {
public:
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef NodeData NodeDataType;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;

    typedef QueryHandler<SimulationTraits> QueryHandlerType;
    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef typename RTreeNodeType::RootReplacedByChildCallback RootReplacedByChildCallback;
    typedef typename RTreeNodeType::NodeSplitCallback NodeSplitCallback;

    typedef typename RTreeNodeType::LiftCutCallback LiftCutCallback;
    typedef typename RTreeNodeType::ObjectInsertedCallback ObjectInsertedCallback;
    typedef typename RTreeNodeType::ObjectRemovedCallback ObjectRemovedCallback;

    typedef typename RTreeNodeType::Index Index;

    RTree(QueryHandlerType* handler, Index elements_per_node, LocationServiceCacheType* loccache,
        bool static_objects,
        AggregateListenerType* agg = NULL,
        RootReplacedByChildCallback root_replaced_cb = 0, NodeSplitCallback node_split_cb = 0,
        LiftCutCallback lift_cut_cb = 0, ObjectInsertedCallback obj_ins_cb = 0, ObjectRemovedCallback obj_rem_cb = 0
    )
     : mLocCache(loccache),
       mStaticObjects(static_objects),
       mRestructureMightHaveEffect(false)
    {
        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;

        mCallbacks.handler = handler;
        mCallbacks.aggregate = agg;
        mCallbacks.objectLeafChanged = std::tr1::bind(&RTree::onObjectLeafChanged, this, _1, _2);
        mCallbacks.getObjectLeaf = std::tr1::bind(&RTree::getObjectLeaf, this, _1);
        mCallbacks.rootReplaced = root_replaced_cb;
        mCallbacks.nodeSplit = node_split_cb;
        mCallbacks.liftCut = lift_cut_cb;
        mCallbacks.objectInserted = obj_ins_cb;
        mCallbacks.objectRemoved = obj_rem_cb;

        mRoot = new RTreeNodeType(elements_per_node, mCallbacks);
    }

    ~RTree() {
    }


    RTreeNodeType* root() {
        return mRoot;
    }
    const RTreeNodeType* root() const {
        return mRoot;
    }

    int size() const { return mRoot->treeSize(); }

    void insert(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) == mObjectLeaves.end());
        mObjectLeaves[objid] = NULL;

        mRoot = RTree_insert_object(mRoot, mLocCache, obj, t, mCallbacks);

        mRestructureMightHaveEffect = true;
    }

    void update(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_update_object(mRoot, mLocCache, objid, t, mCallbacks);

        mRestructureMightHaveEffect = true;
    }

    void update(const Time& t) {
        if (!mStaticObjects)
            mRoot = RTree_update_tree(mRoot, mLocCache, t, mCallbacks);
    }

    void reportBounds(const Time& t) {
        RTree_report_bounds(stdout, mRoot, mLocCache, t);
        fprintf(stdout, "\n");
    }

    void restructure(const Time& t) {
        // Only restructure if we're dealing with dynamic objects or if
        // something has changed/the last restructure pass did something
        if (!mStaticObjects || mRestructureMightHaveEffect) {
            RestructureInfo info = RTree_restructure_tree(mRoot, mLocCache, t, mCallbacks);
            if (mCallbacks.handler->reportRestructures())
                printf("{ \"time\" : %d, \"count\" : %d, \"cuts-rebuilt\" : %d }\n", (int)(t-Time::null()).milliseconds(), info.restructures, info.cutRebuilds);
            // Without any additions/removals, a restructure pass can only have
            // an effect if this previous pass actually restructured something.
            mRestructureMightHaveEffect = (info.restructures > 0);
        }
    }

    void erase(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_delete_object(mRoot, mLocCache, obj, t, mCallbacks);
        mObjectLeaves.erase(objid);

        mRestructureMightHaveEffect = true;
    }

    /** If in debug mode, verify the constraints on the data structure. */
    void verifyConstraints(const Time& t) {
        RTree_verify_constraints(mRoot, mLocCache, t);
    }

    /** Perform a full rebuild of the tree. */
    void rebuild(const Time& t) {
        RTreeNodeType* newRoot = RTree_rebuild(
            mRoot, mLocCache, t,
            objectsBegin(), objectsEnd(), objectsSize(),
            mCallbacks
        );
        RTree_destroy_tree(mRoot, mLocCache, mCallbacks);
        mRoot = newRoot;
        mRestructureMightHaveEffect = true;
    }

private:
    typedef std::tr1::unordered_map<ObjectID, RTreeNodeType*, ObjectIDHasher> ObjectLeafIndex;

    template<class InternalIterator>
    class IteratorWrapper {
    public:
        IteratorWrapper(InternalIterator _it)
         :it(_it)
        {}

        IteratorWrapper& operator++() {
            it++;
            return *this;
        }
        IteratorWrapper& operator++(int) {
            it++;
            return *this;
        }

        const ObjectID& id() const { return it->first; }
        const ObjectID& operator*() const { return it->first; }

        bool operator==(const IteratorWrapper& rhs) {
            return it == rhs.it;
        }
        bool operator!=(const IteratorWrapper& rhs) {
            return it != rhs.it;
        }
    private:
        InternalIterator it;
    };
    typedef IteratorWrapper<typename ObjectLeafIndex::iterator> ObjectIDIterator;
    typedef IteratorWrapper<typename ObjectLeafIndex::const_iterator> ConstObjectIDIterator;

    ObjectIDIterator objectsBegin() { return ObjectIDIterator(mObjectLeaves.begin()); }
    ObjectIDIterator objectsEnd() { return ObjectIDIterator(mObjectLeaves.end()); }
    int objectsSize() { return mObjectLeaves.size(); }

    void onObjectLeafChanged(const LocCacheIterator& obj, RTreeNodeType* node) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());
        assert(node != NULL);
        mObjectLeaves[objid] = node;
    }

    RTreeNodeType* getObjectLeaf(const ObjectID& objid) {
        typename ObjectLeafIndex::const_iterator it = mObjectLeaves.find(objid);
        if (it == mObjectLeaves.end()) return NULL;
        return it->second;
    }

    LocationServiceCacheType* mLocCache;
    RTreeNodeType* mRoot;
    typename RTreeNodeType::Callbacks mCallbacks;
    ObjectLeafIndex mObjectLeaves;
    bool mStaticObjects;
    bool mRestructureMightHaveEffect;
}; // class RTree

} // namespace Prox

#endif //_PROX_RTREE_HPP_
