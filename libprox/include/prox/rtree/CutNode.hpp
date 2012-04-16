// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_RTREE_CUT_NODE_HPP_
#define _PROX_RTREE_CUT_NODE_HPP_

namespace Prox {

template <typename SimulationTraits, typename QueryHandlerTypeT, typename NodeDataTypeT, typename CutTypeT, typename CutNodeTypeT>
struct CutNodeBase {
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef QueryHandlerTypeT QueryHandlerType;
    typedef NodeDataTypeT NodeDataType;
    typedef CutTypeT CutType;
    typedef CutNodeTypeT CutNodeType;
    typedef std::pair<CutNodeType*, CutNodeType*> RangeType;

    CutType* parent;
    typedef typename Prox::RTree<SimulationTraits, NodeDataType, CutNodeType>::RTreeNodeType RTreeNodeType;
    RTreeNodeType * rtnode;
    bool satisfies;

    CutNodeBase(QueryHandlerType* handler, CutType* _parent, RTreeNodeType* _rt, AggregateListenerType* listener)
     : parent(_parent),
       rtnode(_rt),
       satisfies(false)
    {
        rtnode->insertCutNode(getNativeThis());
        if (listener != NULL) listener->aggregateObserved(handler, rtnode->aggregateID(), rtnode->cutNodesSize());
    }

    CutType* getParent() const { return parent; }

    void destroy(QueryHandlerType* handler, AggregateListenerType* listener) {
        rtnode->eraseCutNode(getNativeThis());
        if (listener != NULL) listener->aggregateObserved(handler, rtnode->aggregateID(), rtnode->cutNodesSize());
        delete getNativeThis();
    }

protected:
    ~CutNodeBase() {
    }

    // Gets the this pointer in the native CutNode type, i.e. the real type of
    // this class rather than as this base class.
    CutNodeType* getNativeThis() {
        return static_cast<CutNodeType*>(this);
    }
public:

    bool updateSatisfies(const Vector3& qpos, const BoundingSphere& qregion, float qmaxsize, const SolidAngle& qangle, float qradius) {
        satisfies = rtnode->data().satisfiesConstraints(qpos, qregion, qmaxsize, qangle, qradius);
        return satisfies;
    }

    bool leaf() const {
        return rtnode->leaf();
    }

    void handleRootReplaced(RTreeNodeType* orig_root, RTreeNodeType* new_root) {
        parent->handleRootReplaced(getNativeThis(), orig_root, new_root);
    }
    void handleRootCreatedAboveCut(RTreeNodeType* orig_root, RTreeNodeType* new_root) {
        parent->handleRootCreatedAboveCut(getNativeThis(), orig_root, new_root);
    }
    void handleRootDestroyedAboveCut(RTreeNodeType* orig_root, RTreeNodeType* new_root) {
        parent->handleRootDestroyedAboveCut(getNativeThis(), orig_root, new_root);
    }
    void handleSplit(RTreeNodeType* orig_node, RTreeNodeType* new_node) {
        parent->handleSplit(getNativeThis(), orig_node, new_node);
    }
    void handleSplitAboveCut(RTreeNodeType* orig_node, RTreeNodeType* new_node) {
        parent->handleSplitAboveCut(getNativeThis(), orig_node, new_node);
    }
    void handleLiftCut(RTreeNodeType* to_node) {
        parent->handleLiftCut(getNativeThis(), to_node);
    }
    void handleObjectInserted(const LocCacheIterator& objit, int objidx) {
        parent->handleObjectInserted(getNativeThis(), objit, objidx);
    }
    void handleObjectRemoved(const LocCacheIterator& objit, bool permanent) {
        parent->handleObjectRemoved(getNativeThis(), objit, permanent);
    }
};


} // namespace Prox

#endif // _PROX_RTREE_CUT_NODE_HPP_
