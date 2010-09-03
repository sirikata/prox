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

#define RTREE_BOUNDS_EPSILON 0.1f // FIXME how should we choose this epsilon?

namespace Prox {

template<typename CutNode>
struct CutNodeContainer {
    typedef std::tr1::unordered_set<CutNode> CutNodeList;
    CutNodeList cuts;

    typedef typename CutNodeList::iterator CutNodeListIterator;
    typedef typename CutNodeList::const_iterator CutNodeListConstIterator;
    CutNodeListIterator cutNodesBegin() { return cuts.begin(); }
    CutNodeListConstIterator cutNodesBegin() const { return cuts.begin(); }
    CutNodeListIterator cutNodesEnd() { return cuts.end(); }
    CutNodeListConstIterator cutNodesEnd() const { return cuts.end(); }

    void insertCutNode(const CutNode& cn) {
        cuts.insert(cn);
    }
    void eraseCutNode(const CutNode& cn) {
        cuts.erase(cn);
    }
    size_t cutNodesSize() const {
        return cuts.size();
    }
    bool cutNodesEmpty() const {
        return cuts.empty();
    }
    CutNodeListIterator findCutNode(const CutNode& cn) {
        return cuts.find(cn);
    }
    CutNodeListConstIterator findCutNode(const CutNode& cn) const {
        return cuts.find(cn);
    }
};

template<typename SimulationTraits, typename CutNode>
struct RTreeLeafNode : public CutNodeContainer<CutNode> {
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    LocCacheIterator object;

    RTreeLeafNode(LocCacheIterator it)
     : object(it)
    {}

};

template<typename SimulationTraits, typename NodeData, typename CutNode>
struct RTreeNode : public CutNodeContainer<CutNode> {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef typename SimulationTraits::ObjectIDRandomType ObjectIDRandom;
    typedef typename SimulationTraits::TimeType Time;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef RTreeLeafNode<SimulationTraits, CutNode> LeafNode;

    typedef std::tr1::function<void(const LocCacheIterator&, RTreeNode*)> ObjectLeafChangedCallback;
    typedef std::tr1::function<RTreeNode*(const ObjectID&)> GetObjectLeafCallback;
    // The root was left with only one child node, cuts on the old root need to
    // "refine" themselves to the new node by simple replacement.  Parameters
    // are the old root followed by the new root.
    typedef std::tr1::function<void(CutNode, RTreeNode*, RTreeNode*)> RootReplacedByChildCallback;
    // CutNode that split occurred at, the node that was split, the new node.
    typedef std::tr1::function<void(CutNode, RTreeNode*, RTreeNode*)> NodeSplitCallback;
    typedef std::tr1::function<void(CutNode, RTreeNode*)> LiftCutCallback;
    typedef std::tr1::function<void(CutNode, const LocCacheIterator&, int)> ObjectInsertedCallback;
    typedef std::tr1::function<void(CutNode, const LocCacheIterator&)> ObjectRemovedCallback;

    typedef uint16 Index;

    struct Callbacks {
        AggregateListenerType* aggregate;
        ObjectLeafChangedCallback objectLeafChanged;
        GetObjectLeafCallback getObjectLeaf;
        RootReplacedByChildCallback rootReplaced;
        NodeSplitCallback nodeSplit;
        LiftCutCallback liftCut;
        ObjectInsertedCallback objectInserted;
        ObjectRemovedCallback objectRemoved;
    };
private:
    static const uint8 LeafFlag = 0x02; // elements are object pointers instead of node pointers

    union {
        RTreeNode** nodes;
        LeafNode* objects;
        uint8* magic;
    } elements;
    RTreeNode* mParent;
    NodeData mData;
    uint8 flags;
    Index count;
    Index max_elements;
    ObjectID aggregate;

public:
    struct NodeChildOperations {
        RTreeNode* child(RTreeNode* parent, int idx) {
            return parent->node(idx);
        }

        RTreeNode* childData(RTreeNode* parent, int idx) {
            return parent->node(idx);
        }

        NodeData data(const LocationServiceCacheType* loc, RTreeNode* child, const Time& ) {
            return child->data();
        }

        void insert(RTreeNode* parent, const LocationServiceCacheType* loc, RTreeNode* newchild, const Time&, const Callbacks& cb) {
            parent->insert(newchild, cb);
        }
    };

    struct ObjectChildOperations {
        const LeafNode& child(RTreeNode* parent, int idx) {
            return parent->object(idx);
        }

        const LocCacheIterator& childData(RTreeNode* parent, int idx) {
            return parent->object(idx).object;
        }

        NodeData data(const LocationServiceCacheType* loc, const LocCacheIterator& child, const Time& t) {
            return NodeData(loc, child, t);
        }

        void insert(RTreeNode* parent, const LocationServiceCacheType* loc, const LocCacheIterator& newchild, const Time& t, const Callbacks& cb) {
            parent->insert(loc, newchild, t, cb);
        }
    };


    RTreeNode(Index _max_elements, const Callbacks& callbacks)
     : mParent(NULL), mData(), flags(0), count(0), max_elements(_max_elements), aggregate( ObjectIDRandom()() )
    {
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(LeafNode) );
        uint32 magic_size = max_element_size * max_elements;
        elements.magic = new uint8[magic_size];
        memset(elements.magic, 0, magic_size);

        leaf(true);

        if (callbacks.aggregate != NULL) callbacks.aggregate->aggregateCreated(aggregate);
    }

    // We have a destroy method and hide the destructor in private in order to
    // ensure the aggregate callbacks get invoked properly.
    void destroy(const Callbacks& callbacks) {
        if (callbacks.aggregate != NULL) callbacks.aggregate->aggregateDestroyed(aggregate);
        delete this;
    }

private:
    ~RTreeNode() {
        delete[] elements.magic;
        assert(CutNodeContainer<CutNode>::cuts.size() == 0);
    }

public:
    bool leaf() const {
        return (flags & LeafFlag);
    }
    void leaf(bool d) {
        flags = (flags & ~LeafFlag) | (d ? LeafFlag : 0x00);
    }

    bool empty() const {
        return (count == 0);
    }
    bool full() const {
        return (count == max_elements);
    }
    Index size() const {
        return count;
    }
    Index capacity() const {
        return max_elements;
    }

    const ObjectID& aggregateID() const { return aggregate; }

    RTreeNode* parent() const {
        return mParent;
    }
    void parent(RTreeNode* _p) {
        mParent = _p;
    }

    const LeafNode& object(int i) const {
        assert( leaf() );
        assert( i < count );
        return elements.objects[i];
    }

    RTreeNode* node(int i) const {
        assert( !leaf() );
        assert( i < count );
        return elements.nodes[i];
    }

    const NodeData& data() const {
        return mData;
    }

    NodeData childData(int i, const LocationServiceCacheType* loc, const Time& t) {
        if (leaf())
            return NodeData(loc, object(i).object, t);
        else
            return node(i)->data();
    }

    void recomputeData(const LocationServiceCacheType* loc, const Time& t, const Callbacks& cb) {
        mData = NodeData();
        for(int i = 0; i < size(); i++)
            mData.mergeIn( childData(i, loc, t) );
        if (cb.aggregate != NULL) cb.aggregate->aggregateBoundsUpdated(aggregate, mData.getBounds());
    }

    void clear() {
        count = 0;

        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(LeafNode) );
        for(uint32 i = 0; i < max_element_size * max_elements; i++)
            elements.magic[i] = 0;

        mData = NodeData();
    }

    void insert(const LocationServiceCacheType* loc, const LocCacheIterator& obj, const Time& t, const Callbacks& cb) {
        assert (count < max_elements);
        assert (leaf() == true);

        int idx = count;
        elements.objects[idx] = LeafNode(obj);
        cb.objectLeafChanged(obj, this);
        count++;
        mData.mergeIn( NodeData(loc, obj, t) );

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildAdded(aggregate, loc->iteratorID(obj), mData.getBounds());

        if (cb.objectInserted) {
            for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = this->cutNodesBegin(); cut_it != this->cutNodesEnd(); cut_it++)
                cb.objectInserted(*cut_it, obj, idx);
        }
    }

    void insert(RTreeNode* node, const Callbacks& cb) {
        assert (count < max_elements);
        assert (leaf() == false);
        node->parent(this);
        elements.nodes[count] = node;
        count++;
        mData.mergeIn( node->data() );

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildAdded(aggregate, node->aggregate, mData.getBounds());
    }

    // NOTE: does not recalculate the bounding sphere
    void erase(const LocationServiceCacheType* loc, const LocCacheIterator& obj, const Callbacks& cb) {
        assert(count > 0);
        assert(leaf() == true);

        // find obj
        Index obj_idx;
        for(obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx].object == obj) break;
        // push all the other objects back one
        for(Index rem_idx = obj_idx; rem_idx < count-1; rem_idx++)
            elements.objects[rem_idx] = elements.objects[rem_idx+1];
        count--;

        ObjectID obj_id = loc->iteratorID(obj);
        if (cb.objectRemoved) {
            for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = this->cutNodesBegin(); cut_it != this->cutNodesEnd(); cut_it++)
                cb.objectRemoved(*cut_it, obj);
        }

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildRemoved(aggregate, obj_id, mData.getBounds());
    }

private:
    // NOTE: does not recalculate the bounding sphere.
    // Internal helper.  Doesn't notify aggregates.
    void erase(int node_idx) {
        // push all the other objects back one
        for(Index rem_idx = node_idx; rem_idx < count-1; rem_idx++)
            elements.nodes[rem_idx] = elements.nodes[rem_idx+1];
        count--;
    }
public:
    // NOTE: does not recalculate the bounding sphere
    void erase(const RTreeNode* node, const Callbacks& cb) {
        assert(count > 0);
        assert(leaf() == false);
        // find node
        Index node_idx;
        for(node_idx = 0; node_idx < count; node_idx++)
            if (elements.nodes[node_idx] == node) break;
        erase(node_idx);

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildRemoved(aggregate, node->aggregate, mData.getBounds());
    }

    bool contains(const LocCacheIterator& obj) const {
        for(Index obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx] == obj) return true;
        return false;
    }

    RTreeNode* selectBestChildNode(const LocationServiceCacheType* loc, const LocCacheIterator& obj, const Time& t) {
        return NodeData::selectBestChildNode(this, loc, obj, t);
    }

};


static const int32 UnassignedGroup = -1;
//typedef std::vector<NodeData> SplitData;
typedef std::vector<int32> SplitGroups;


template<typename SimulationTraits, typename NodeData, typename CutNode>
class BoundingSphereDataBase {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;
    typedef Query<SimulationTraits> QueryType;

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    BoundingSphereDataBase()
     : bounding_sphere()
    {
    }

    BoundingSphereDataBase(const LocationServiceCacheType* loc, const LocCacheIterator& obj, const Time& t)
     : bounding_sphere( loc->worldCompleteBounds(obj, t) )
    {
        // Note use of worldCompleteBounds above instead of worldRegion because
        // we need to take into account the size of the objects as well as their
        // locations.
    }

    // Return the result of merging this info with the given info
    NodeData merge(const NodeData& other) const {
        NodeData result;
        result.bounding_sphere = bounding_sphere.merge(other.bounding_sphere);
        return result;
    }

    // Merge the given info into this info
    void mergeIn(const NodeData& other) {
        bounding_sphere.mergeIn(other.bounding_sphere);
    }

    // Check if this data satisfies the query constraints given
    bool satisfiesConstraints(const Vector3& qpos, const BoundingSphere& qregion, const float qmaxsize, const SolidAngle& qangle, const float qradius) const {
        Vector3 obj_pos = bounding_sphere.center();
        float obj_radius = bounding_sphere.radius();

        return satisfiesConstraintsBounds<SimulationTraits>(obj_pos, obj_radius, qpos, qregion, qmaxsize, qangle, qradius);
    }

    // Given an object and a time, select the best child node to put the object in
    static RTreeNodeType* selectBestChildNode(const RTreeNodeType* node, const LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
        float min_increase = 0.f;
        RTreeNodeType* min_increase_node = NULL;

        BoundingSphere obj_bounds = loc->worldCompleteBounds(obj_id, t);

        for(int i = 0; i < node->size(); i++) {
            RTreeNodeType* child_node = node->node(i);
            BoundingSphere merged = child_node->data().bounding_sphere.merge(obj_bounds);
            float increase = merged.volume() - child_node->data().bounding_sphere.volume();
            if (min_increase_node == NULL || increase < min_increase) {
                min_increase = increase;
                min_increase_node = child_node;
            }
        }

        return min_increase_node;
    }

    // Given a list of child data, choose two seeds for the splitting process in quadratic time
    static void pickSeedsQuadratic(const std::vector<NodeData>& split_data, int32* seed0, int32* seed1) {
        *seed0 = -1; *seed1 = -1;
        float max_waste = -FLT_MAX;
        for(uint32 idx0 = 0; idx0 < split_data.size(); idx0++) {
            for(uint32 idx1 = idx0+1; idx1 < split_data.size(); idx1++) {
                BoundingSphere merged = split_data[idx0].bounding_sphere.merge(split_data[idx1].bounding_sphere);

                float waste = merged.volume() - split_data[idx0].bounding_sphere.volume() - split_data[idx1].bounding_sphere.volume();

                if (waste > max_waste) {
                    max_waste = waste;
                    *seed0 = idx0;
                    *seed1 = idx1;
                }
            }
        }
    }

    // Given list of split data and current group assignments as well as current group data, select the next child to be added and its group
    static void pickNextChild(std::vector<NodeData>& split_data, const SplitGroups& split_groups, const NodeData& group_data_0, const NodeData& group_data_1, int32* next_child, int32* selected_group) {
        float max_preference = -1.0f;
        *next_child = -1;
        *selected_group = -1;

        for(uint32 i = 0; i < split_data.size(); i++) {
            if (split_groups[i] != UnassignedGroup) continue;

            BoundingSphere merged0 = group_data_0.bounding_sphere.merge(split_data[i].bounding_sphere);
            BoundingSphere merged1 = group_data_1.bounding_sphere.merge(split_data[i].bounding_sphere);

            float diff0 = merged0.volume() - split_data[i].bounding_sphere.volume();
            float diff1 = merged1.volume() - split_data[i].bounding_sphere.volume();

            float preference = fabs(diff0 - diff1);
            if (preference > max_preference) {
                max_preference = preference;
                *next_child = i;
                *selected_group = (diff0 < diff1) ? 0 : 1;
            }
        }
    }

    void verifyChild(const NodeData& child) const {
        if (! bounding_sphere.contains( child.bounding_sphere, RTREE_BOUNDS_EPSILON )) {
            printf("child exceeds bounds %f\n",
                bounding_sphere.radius() - ((bounding_sphere.center() - child.bounding_sphere.center()).length() + child.bounding_sphere.radius())
            );
        }
    }

    /** Gets the current bounds of the node.  This should be the true,
     *  static bounds of the objects, not just the region they cover.
     *  Use of this method (to generate aggregate object Loc
     *  information) currently assumes we're not making aggregates be
     *  moving objects.
     */
    BoundingSphere getBounds() const {
        return bounding_sphere;
    }
protected:
    BoundingSphere bounding_sphere;
};


template<typename SimulationTraits, typename CutNode>
class BoundingSphereData : public BoundingSphereDataBase<SimulationTraits, BoundingSphereData<SimulationTraits, CutNode>, CutNode> {
public:
    typedef BoundingSphereDataBase<SimulationTraits, BoundingSphereData<SimulationTraits, CutNode>, CutNode> ThisBase;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    BoundingSphereData()
     : BoundingSphereDataBase<SimulationTraits, BoundingSphereData, CutNode>()
    {
    }

    BoundingSphereData(const LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t)
     : BoundingSphereDataBase<SimulationTraits, BoundingSphereData, CutNode>( loc, obj_id, t )
    {
    }
};

/* Maintains the largest bounding sphere radius as well as the hierarchical bounding sphere.
 * We can cull if the largest bounding sphere, centered at the closest point on the
 * hierarchical bounding sphere, does not satisfy the constraints.
 */
template<typename SimulationTraits, typename CutNode>
class MaxSphereData : public BoundingSphereDataBase<SimulationTraits, MaxSphereData<SimulationTraits, CutNode>, CutNode> {
public:
    typedef MaxSphereData NodeData; // For convenience/consistency
    typedef BoundingSphereDataBase<SimulationTraits, MaxSphereData<SimulationTraits, CutNode>, CutNode> ThisBase;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::TimeType Time;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef Query<SimulationTraits> QueryType;

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    MaxSphereData()
     : BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>(),
       mMaxRadius(0.f)
    {
    }

    MaxSphereData(const LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t)
     : BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>(),
       mMaxRadius( loc->maxSize(obj_id) )
    {
        // Note: we override this here because we need worldCompleteBounds for
        // just the bounds data, but with the max size values, we can use the
        // smaller worldRegion along with the maximum size object.  Note
        // difference in satisfiesConstraints
        ThisBase::bounding_sphere = loc->worldRegion(obj_id, t);
    }

    NodeData merge(const NodeData& other) const {
        NodeData result = BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>::merge(other);
        result.mMaxRadius = std::max( mMaxRadius, other.mMaxRadius );
        return result;
    }

    // Merge the given info into this info
    void mergeIn(const NodeData& other) {
        BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>::mergeIn(other);
        mMaxRadius = std::max( mMaxRadius, other.mMaxRadius );
    }

    // Check if this data satisfies the query constraints given
    bool satisfiesConstraints(const Vector3& qpos, const BoundingSphere& qregion, const float qmaxsize, const SolidAngle& qangle, const float qradius) const {
        // We create a virtual stand in object which is the worst case object that could be in this subtree.
        // It's centered at the closest point on the hierarchical bounding sphere to the query, and has the
        // largest radius of any objects in the subtree.

        Vector3 obj_pos = ThisBase::bounding_sphere.center();
        float obj_radius = ThisBase::bounding_sphere.radius();

        return satisfiesConstraintsBoundsAndMaxSize<SimulationTraits>(obj_pos, obj_radius, mMaxRadius, qpos, qregion, qmaxsize, qangle, qradius);
    }

    // Given an object and a time, select the best child node to put the object in
    static RTreeNodeType* selectBestChildNode(const RTreeNodeType* node, const LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
        float min_increase = 0.f;
        RTreeNodeType* min_increase_node = NULL;

        BoundingSphere obj_bounds = loc->worldRegion(obj_id, t);
        float obj_max_size = loc->maxSize(obj_id);

        for(int i = 0; i < node->size(); i++) {
            RTreeNodeType* child_node = node->node(i);
            BoundingSphere merged = child_node->data().bounding_sphere.merge(obj_bounds);
            float new_max_size = std::max(child_node->data().mMaxRadius, obj_max_size);
            BoundingSphere old_total( child_node->data().bounding_sphere.center(), child_node->data().bounding_sphere.radius() + child_node->data().mMaxRadius );
            BoundingSphere total(merged.center(), merged.radius() + new_max_size);
            float increase = total.volume() - old_total.volume();
            if (min_increase_node == NULL || increase < min_increase) {
                min_increase = increase;
                min_increase_node = child_node;
            }
        }

        return min_increase_node;
    }

    void verifyChild(const NodeData& child) const {
        BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>::verifyChild(child);

        if ( child.mMaxRadius > mMaxRadius) {
            printf(
                "Child radius greater than recorded maximum child radius: %f > %f\n",
                child.mMaxRadius, mMaxRadius
            );
        }
    }

    /** Gets the current bounds of the node.  This should be the true,
     *  static bounds of the objects, not just the region they cover.
     *  Use of this method (to generate aggregate object Loc
     *  information) currently assumes we're not making aggregates be
     *  moving objects.
     */
    BoundingSphere getBounds() const {
        return BoundingSphere( ThisBase::bounding_sphere.center(), ThisBase::bounding_sphere.radius() + mMaxRadius );
    }
private:
    float mMaxRadius;
};

template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_choose_leaf(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t)
{
    RTreeNode<SimulationTraits, NodeData, CutNode>* node = root;

    while(!node->leaf()) {
        RTreeNode<SimulationTraits, NodeData, CutNode>* min_increase_node = node->selectBestChildNode(loc, obj_id, t);
        assert(min_increase_node != NULL);
        node = min_increase_node;
    }

    return node;
}

// Quadratic algorithm for picking node split seeds
template<typename NodeData>
void RTree_quadratic_pick_seeds(const std::vector<NodeData>& split_data, SplitGroups& split_groups, NodeData& group_data_0, NodeData& group_data_1) {
    int32 seed0 = -1, seed1 = -1;
    NodeData::pickSeedsQuadratic(split_data, &seed0, &seed1);
    assert( seed0 != -1 && seed1 != -1 );

    split_groups[seed0] = 0;
    split_groups[seed1] = 1;
    group_data_0 = split_data[seed0];
    group_data_1 = split_data[seed1];
}

// Choose the next child to assign to a group
template<typename NodeData>
void RTree_pick_next_child(std::vector<NodeData>& split_data, SplitGroups& split_groups, NodeData& group_data_0, NodeData& group_data_1) {
    int32 next_child = -1;
    int32 selected_group = -1;
    NodeData::pickNextChild( split_data, split_groups, group_data_0, group_data_1, &next_child, &selected_group);
    assert(next_child != -1);
    assert(selected_group != -1);

    split_groups[next_child] = selected_group;
    if (selected_group == 0)
        group_data_0.mergeIn(split_data[next_child]);
    else
        group_data_1.mergeIn(split_data[next_child]);

    return;
}

// Splits a node, inserting the given node, and returns the second new node
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_split_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    ChildType to_insert,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    /** Since we're splitting node, we need to lift any cuts up to its
     * parent. If its the root, just lift to the node itself and the
     * notification of splits should take care of getting the cut right.
     */
    RTreeNodeType* parent = node->parent();
    if (parent)
        RTree_lift_cut_nodes(node, parent, cb);
    else
        RTree_lift_cut_nodes(node, node, cb);
    RTree_verify_no_cut_nodes(node);


    ChildOperations child_ops;

    // collect the info for the children
    std::vector<ChildType> split_children;
    std::vector<NodeData> split_data;
    SplitGroups split_groups;

    // add all the children to the split vectors
    for(int i = 0; i < node->size(); i++) {
        split_children.push_back( child_ops.childData(node, i) );
        split_data.push_back( node->childData(i,loc,t) );
        split_groups.push_back(UnassignedGroup);
    }
    split_children.push_back( to_insert );
    split_data.push_back( child_ops.data(loc, to_insert, t) );
    split_groups.push_back(UnassignedGroup);

    // find the initial seeds
    NodeData group_data_0, group_data_1;
    RTree_quadratic_pick_seeds(split_data, split_groups, group_data_0, group_data_1);

    // group the remaining ones
    for(uint32 i = 0; i < split_children.size()-2; i++)
        RTree_pick_next_child(split_data, split_groups, group_data_0, group_data_1);

    // copy data into the correct nodes
    node->clear();
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn = new RTreeNode<SimulationTraits, NodeData, CutNode>(node->capacity(), cb);
    nn->leaf(node->leaf());
    for(uint32 i = 0; i < split_children.size(); i++) {
        RTreeNode<SimulationTraits, NodeData, CutNode>* newparent = (split_groups[i] == 0) ? node : nn;
        child_ops.insert( newparent, loc, split_children[i], t, cb);
    }
    // Notify the cuts of the split so it can be updated. Either all have been
    // lifted up to this node if its the root or there are none because they
    // have been lifted to the parent.  There should be no cuts in the children
    // nodes.
    if (cb.nodeSplit) {
        for(typename RTreeNodeType::CutNodeListConstIterator cut_it = node->cutNodesBegin(); cut_it != node->cutNodesEnd(); cut_it++) {
            CutNode cutnode = *cut_it;
            cb.nodeSplit(cutnode, node, nn);
        }
    }


    return nn;
}

// Fixes up the tree after insertion. Returns the new root node
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_adjust_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* L,
    RTreeNode<SimulationTraits, NodeData, CutNode>* LL,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
 {
    assert(L->leaf());
    RTreeNode<SimulationTraits, NodeData, CutNode>* node = L;
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn = LL;

    while(true) { // loop until root, enter for root as well so we recompute its bounds
        RTreeNode<SimulationTraits, NodeData, CutNode>* parent = node->parent();

        // FIXME this is inefficient
        node->recomputeData(loc, t, cb);

        if (parent == NULL) break;

        RTreeNode<SimulationTraits, NodeData, CutNode>* pp = NULL;
        if (nn != NULL) {
            if (parent->full())
                pp = RTree_split_node<SimulationTraits, NodeData, CutNode, RTreeNode<SimulationTraits, NodeData, CutNode>*, typename RTreeNode<SimulationTraits, NodeData, CutNode>::NodeChildOperations>(parent, nn, loc, t, cb);
            else
                parent->insert(nn, cb);
        }

        node = parent;
        nn = pp;
    }

    // if we have a leftover split node, the root was split and we need to create
    // a new root one level higher
    if (nn != NULL) {
        RTreeNode<SimulationTraits, NodeData, CutNode>* new_root = new RTreeNode<SimulationTraits, NodeData, CutNode>(node->capacity(), cb);
        new_root->leaf(false);
        new_root->insert(node, cb);
        new_root->insert(nn, cb);

        node = new_root;
        nn = NULL;
    }

    return node;
}

// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_insert_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_node = RTree_choose_leaf(root, loc, obj_id, t);

    RTreeNode<SimulationTraits, NodeData, CutNode>* split_node = NULL;
    if (leaf_node->full())
        split_node = RTree_split_node<SimulationTraits, NodeData, CutNode, typename LocationServiceCache<SimulationTraits>::Iterator, typename RTreeNode<SimulationTraits, NodeData, CutNode>::ObjectChildOperations>(leaf_node, obj_id, loc, t, cb);
    else
        leaf_node->insert(loc, obj_id, t, cb);

    RTreeNode<SimulationTraits, NodeData, CutNode>* new_root = RTree_adjust_tree(leaf_node, split_node, loc, t, cb);

    return new_root;
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
int32 RTree_count(RTreeNode<SimulationTraits, NodeData, CutNode>* root) {
    if (root->leaf()) return root->size();

    int32 result = 0;
    for(int i = 0; i < root->size(); i++)
        result += RTree_count(root->node(i));
    return result;
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_verify_constraints(RTreeNode<SimulationTraits, NodeData, CutNode>* root, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::TimeType& t) {
#ifdef PROXDEBUG
    for(int i = 0; i < root->size(); i++) {
        if(!root->leaf()) {
            assert(root->node(i)->parent() == root);
        }
        root->data().verifyChild( root->childData(i, loc, t) );
    }
    if (!root->leaf()) {
        for(int i = 0; i < root->size(); i++)
            RTree_verify_constraints(root->node(i), loc, t);
    }
#endif // def PROXDEBUG
}


/** Get all cuts in from_node and its children to "lift" themselves up to
 *  to_node so the subtree can be operated on.  This may require the cuts
 *  adjusting other subtrees as well, so the tree should be in a clean state
 *  before calling this.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_lift_cut_nodes(
    RTreeNode<SimulationTraits, NodeData, CutNode>* from_node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* to_node,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename RTreeNodeType::Index Index;

    // Only notify cuts if we're moving *above* this node, not if this node is
    // both the starting point for recursion and the destination node
    if (from_node != to_node) {
        // Notify cuts to lift up to the right node
        // NOTE: We use this approach since the callback likely adjusts the cut
        // node list in from_node
        while(!from_node->cutNodesEmpty())
            cb.liftCut(*from_node->cutNodesBegin(), to_node);
    }

    // And recurse
    if (!from_node->leaf()) {
        for(Index idx = 0; idx < from_node->size(); idx++) {
            RTree_lift_cut_nodes(from_node->node(idx), to_node, cb);
        }
    }
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_verify_no_cut_nodes(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
#ifdef PROXDEBUG
    assert(node->cutNodesSize() == 0);
    if (!node->leaf())
        for(typename RTreeNodeType::Index idx = 0; idx < node->size(); idx++)
            RTree_verify_no_cut_nodes(node->node(idx));
#endif
}

/* Takes a leaf node from which an object has been removed and, if it contains too few nodes,
 *  redistributes the objects it contains and removes the node from the tree.
 *  Returns the new root (which it may create because it might have to reinsert objects, which
 *  can itself cause a new root to appear.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_condense_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename RTreeNodeType::Index Index;

    RTreeNodeType* n = NULL;
    // Track the highest removed node so we can remove all nodes under it
    RTreeNodeType* highest_removed = NULL;
    // Staring point for recomputing per-node data.  Is leaf by default, but if
    // a subtree is removed, it starts at the removed nodes parent.
    RTreeNodeType* recompute_start = leaf;

    // Track up the tree checking for nodes to condense into their parent
    n = leaf;
    bool last_removed = false; // tracks if child was removed
    while(n->parent() != NULL) {
        RTreeNodeType* parent = n->parent();
        int child_count = last_removed ? n->size()-1 : n->size();
        last_removed = false;
        if (child_count < 1) { // FIXME should be some larger value
            highest_removed = n;
            recompute_start = parent;
            last_removed = true;
        }
        n = parent;
    }

    RTreeNode<SimulationTraits, NodeData, CutNode>* root = n;

    // Handle the removal if one occurred. Note that the tree has yet to change,
    // so we can go through safely give all the cuts notification of the changes.
    if (highest_removed != NULL) {
        RTreeNodeType* parent = highest_removed->parent();

        // First, get any cuts in the subtree to "lift" themselves up to the
        // parent node.
        RTree_lift_cut_nodes(highest_removed, parent, cb);
        RTree_verify_no_cut_nodes(highest_removed);

        // Then, remove nodes
        std::queue<RTreeNodeType*> removedNodes;
        parent->erase(highest_removed, cb);
        removedNodes.push(highest_removed);

        // FIXME this could reinsert entire nodes instead of individual objects, but we'd need a better idea of how to actually accomplish that...
        while(!removedNodes.empty()) {
            RTreeNode<SimulationTraits, NodeData, CutNode>* removed = removedNodes.front();
            removedNodes.pop();

            if (removed->leaf()) {
                for(Index idx = 0; idx < removed->size(); idx++)
                    root = RTree_insert_object(root, loc, removed->object(idx).object, t, cb);
            }
            else {
                while(removed->size()) {
                    RTreeNodeType* child_removed = removed->node(0);
                    removedNodes.push(child_removed);
                    removed->erase(child_removed, cb);
                }
            }
            removed->destroy(cb);
        }
    }


    // After removal, there's a chance that the root node ended up with no
    // elements, in which case it should be marked as a leaf node
    if (root->size() == 0)
        root->leaf(true);

    // Perform recomputation of node data
    n = recompute_start;
    while(n->parent() != NULL) {
        RTreeNode<SimulationTraits, NodeData, CutNode>* parent = n->parent();
        n->recomputeData(loc, t, cb);
        n = parent;
    }

    return root;
}

/* Update nodes up the tree to fix node information due to a change. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_update_up_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn = node;

    while(nn != NULL) {
        nn->recomputeData(loc, t, cb);
        nn = nn->parent();
    }
}

/* Updates the object in the given tree. Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_update_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::ObjectIDType& obj_id,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_with_obj = cb.getObjectLeaf(obj_id);
    if (leaf_with_obj == NULL)
        return root;

    RTree_update_up_tree(leaf_with_obj, loc, t, cb);

    return root;
}

/* Updates objects in the tree with new positions. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_update_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    if (!root->leaf()) {
        for(int i = 0; i < root->size(); i++) {
            // FIXME set update node
            RTree_update_tree(root->node(i), loc, t, cb);
        }
    }

    root->recomputeData(loc, t, cb);

    return root;
}

/* Deletes the object from the given tree.  Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_delete_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;

    ObjectIDType real_obj_id = loc->iteratorID(obj_id);
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_with_obj = cb.getObjectLeaf(real_obj_id);
    if (leaf_with_obj == NULL) {
        return root;
    }

    // Notify any cuts that the object is leaving
    assert(leaf_with_obj->leaf());
    leaf_with_obj->erase(loc, obj_id, cb);

    RTreeNode<SimulationTraits, NodeData, CutNode>* new_root = RTree_condense_tree(leaf_with_obj, loc, t, cb);

    // We might need to shorten the tree if the root is left with only one child.
    if (!root->leaf() && root->size() == 1) {
        new_root = root->node(0);
        // Notify cuts so they can refine to the new root
        if (cb.rootReplaced) {
            for(typename RTreeNodeType::CutNodeListConstIterator cut_it = root->cutNodesBegin(); cut_it != root->cutNodesEnd(); cut_it++) {
                CutNode cutnode = *cut_it;
                cb.rootReplaced(cutnode, root, new_root);
            }
        }
        new_root->parent(NULL);
        root->clear();
        root->destroy(cb);
    }
    return new_root;
}





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

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef typename RTreeNodeType::RootReplacedByChildCallback RootReplacedByChildCallback;
    typedef typename RTreeNodeType::NodeSplitCallback NodeSplitCallback;

    typedef typename RTreeNodeType::LiftCutCallback LiftCutCallback;
    typedef typename RTreeNodeType::ObjectInsertedCallback ObjectInsertedCallback;
    typedef typename RTreeNodeType::ObjectRemovedCallback ObjectRemovedCallback;

    typedef typename RTreeNodeType::Index Index;

    RTree(Index elements_per_node, LocationServiceCacheType* loccache, AggregateListenerType* agg = NULL,
        RootReplacedByChildCallback root_replaced_cb = 0, NodeSplitCallback node_split_cb = 0,
        LiftCutCallback lift_cut_cb = 0, ObjectInsertedCallback obj_ins_cb = 0, ObjectRemovedCallback obj_rem_cb = 0
    )
     : mLocCache(loccache)
    {
        using std::tr1::placeholders::_1;
        using std::tr1::placeholders::_2;

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

    void insert(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) == mObjectLeaves.end());
        mObjectLeaves[objid] = NULL;

        mRoot = RTree_insert_object(mRoot, mLocCache, obj, t, mCallbacks);
    }

    void update(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_update_object(mRoot, mLocCache, objid, t, mCallbacks);
    }

    void update(const Time& t) {
        mRoot = RTree_update_tree(mRoot, mLocCache, t, mCallbacks);
    }

    void erase(const LocCacheIterator& obj, const Time& t) {
        const ObjectID& objid = mLocCache->iteratorID(obj);
        assert(mObjectLeaves.find(objid) != mObjectLeaves.end());

        mRoot = RTree_delete_object(mRoot, mLocCache, obj, t, mCallbacks);
        mObjectLeaves.erase(objid);
    }

    /** If in debug mode, verify the constraints on the data structure. */
    void verifyConstraints(const Time& t) {
        RTree_verify_constraints(mRoot, mLocCache, t);
    }

private:
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
    // Index for object's leaf nodes
    typedef std::tr1::unordered_map<ObjectID, RTreeNodeType*, ObjectIDHasher> ObjectLeafIndex;
    ObjectLeafIndex mObjectLeaves;
}; // class RTree

} // namespace Prox

#endif //_PROX_RTREE_HPP_
