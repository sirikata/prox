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


#include <prox/Platform.hpp>
#include <prox/LocationServiceCache.hpp>
#include <float.h>

#define RTREE_BOUNDS_EPSILON 0.1f // FIXME how should we choose this epsilon?

namespace Prox {

template<typename SimulationTraits, typename NodeData>
struct RTreeNode {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;

private:
    static const uint8 LeafFlag = 0x02; // elements are object pointers instead of node pointers

    union {
        RTreeNode** nodes;
        ObjectID* objects;
        uint8* magic;
    } elements;
    RTreeNode* mParent;
    NodeData mData;
    uint8 flags;
    uint8 count;
    uint8 max_elements;

public:
    struct NodeChildOperations {
        RTreeNode* child(RTreeNode* parent, int idx) {
            return parent->node(idx);
        }

        NodeData data(const LocationServiceCacheType* loc, RTreeNode* child, const Time& ) {
            return child->data();
        }

        void insert(RTreeNode* parent, const LocationServiceCacheType* loc, RTreeNode* newchild, const Time& ) {
            parent->insert(newchild);
        }
    };

    struct ObjectChildOperations {
        const ObjectID& child(RTreeNode* parent, int idx) {
            return parent->object(idx);
        }

        NodeData data(const LocationServiceCacheType* loc, const ObjectID& child, const Time& t) {
            return NodeData(loc, child, t);
        }

        void insert(RTreeNode* parent, const LocationServiceCacheType* loc, const ObjectID& newchild, const Time& t) {
            parent->insert(loc, newchild,t);
        }
    };


    RTreeNode(uint8 _max_elements)
     : mParent(NULL), mData(), flags(0), count(0), max_elements(_max_elements)
    {
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(ObjectID) );
        uint32 magic_size = max_element_size * max_elements;
        elements.magic = new uint8[magic_size];
        memset(elements.magic, 0, magic_size);

        leaf(true);
    }

    ~RTreeNode() {
        delete[] elements.magic;
    }

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
    uint8 size() const {
        return count;
    }
    uint8 capacity() const {
        return max_elements;
    }

    RTreeNode* parent() const {
        return mParent;
    }
    void parent(RTreeNode* _p) {
        mParent = _p;
    }

    const ObjectID& object(int i) const {
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
            return NodeData(loc, object(i), t);
        else
            return node(i)->data();
    }

    void recomputeData(const LocationServiceCacheType* loc, const Time& t) {
        mData = NodeData();
        for(int i = 0; i < size(); i++)
            mData.mergeIn( childData(i, loc, t) );
    }

    void clear() {
        count = 0;
        for(int i = 0; i < max_elements; i++)
            elements.magic[i] = NULL;
        mData = NodeData();
    }

    void insert(const LocationServiceCacheType* loc, const ObjectID& obj, const Time& t) {
        assert (count < max_elements);
        assert (leaf() == true);
        elements.objects[count] = obj;
        count++;
        mData.mergeIn( NodeData(loc, obj, t) );
    }

    void insert(RTreeNode* node) {
        assert (count < max_elements);
        assert (leaf() == false);
        node->parent(this);
        elements.nodes[count] = node;
        count++;
        mData.mergeIn( node->data() );
    }

    // NOTE: does not recalculate the bounding sphere
    void erase(const ObjectID& obj) {
        assert(count > 0);
        assert(leaf() == true);
        // find obj
        uint8 obj_idx;
        for(obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx] == obj) break;
        // push all the other objects back one
        for(uint8 rem_idx = obj_idx; rem_idx < count-1; rem_idx++)
            elements.objects[rem_idx] = elements.objects[rem_idx+1];
        count--;
    }

    // NOTE: does not recalculate the bounding sphere
    void erase(const RTreeNode* node) {
        assert(count > 0);
        assert(leaf() == false);
        // find node
        uint8 node_idx;
        for(node_idx = 0; node_idx < count; node_idx++)
            if (elements.nodes[node_idx] == node) break;
        // push all the other objects back one
        for(uint8 rem_idx = node_idx; rem_idx < count-1; rem_idx++)
            elements.nodes[rem_idx] = elements.nodes[rem_idx+1];
        count--;
    }

    bool contains(const ObjectID& obj) const {
        for(uint8 obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx] == obj) return true;
        return false;
    }

    RTreeNode* selectBestChildNode(const LocationServiceCacheType* loc, const ObjectID& obj, const Time& t) {
        return NodeData::selectBestChildNode(this, loc, obj, t);
    }

    RTreeNode* findLeafWithObject(const LocationServiceCacheType* loc, const ObjectID& obj, const Time& t) {
        return NodeData::findLeafWithObject(this, loc, obj, t);
    }
};


static const int32 UnassignedGroup = -1;
//typedef std::vector<NodeData> SplitData;
typedef std::vector<int32> SplitGroups;


template<typename SimulationTraits, typename NodeData>
class BoundingSphereDataBase {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef Query<SimulationTraits> QueryType;

    typedef RTreeNode<SimulationTraits, NodeData> RTreeNodeType;

    BoundingSphereDataBase()
     : bounding_sphere()
    {
    }

    BoundingSphereDataBase(const LocationServiceCacheType* loc, const ObjectID& obj, const Time& t)
     : bounding_sphere( loc->worldBounds(obj, t) )
    {
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
    bool satisfiesConstraints(const Vector3& qpos, const BoundingSphere& qbounds, const float qradius, const SolidAngle& qangle) {
        Vector3 obj_pos = bounding_sphere.center();
        float obj_radius = bounding_sphere.radius();

        // Must satisfy radius constraint
        if (qradius != QueryType::InfiniteRadius && (obj_pos-qpos).lengthSquared() > qradius*qradius)
            return false;

        // Must satisfy solid angle constraint
        // If it falls inside the query bounds, then it definitely satisfies the solid angle constraint
        // FIXME we do this check manually for now, but BoundingSphere should provide it
        if (qbounds.radius() + obj_radius >= (qpos-obj_pos).length()) {
            return true;
        }
        // Otherwise we need to check the closest possible query position to the object
        Vector3 to_obj = obj_pos - qpos;
        to_obj = to_obj - to_obj.normal() * qbounds.radius();
        SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, obj_radius);

        if (solid_angle >= qangle)
            return true;

        return false;
    }

    // Given an object and a time, select the best child node to put the object in
    static RTreeNodeType* selectBestChildNode(const RTreeNodeType* node, const LocationServiceCacheType* loc, const ObjectID& obj_id, const Time& t) {
        float min_increase = 0.f;
        RTreeNodeType* min_increase_node = NULL;

        BoundingSphere obj_bounds = loc->worldBounds(obj_id, t);

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

    // Given a root node and object and a time, find the leaf node which contains that object
    static RTreeNodeType* findLeafWithObject(RTreeNodeType* node, const LocationServiceCacheType* loc, const ObjectID& obj_id, const Time& t) {
        BoundingSphere bs = loc->worldBounds(obj_id, t);
        return findLeafWithObject(node, loc, obj_id, bs);
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

private:
    static RTreeNodeType* findLeafWithObject(RTreeNodeType* node, const LocationServiceCacheType* loc, const ObjectID& obj_id, const BoundingSphere& bs) {
        // For leaf nodes, simply check against all child objects
        if (node->leaf())
            return (node->contains(obj_id) ? node : NULL);

        // For internal nodes, check against child bounds, and then recursively check child
        for(uint8 child_idx = 0; child_idx < node->size(); child_idx++) {
            RTreeNodeType* child = node->node(child_idx);
            BoundingSphere child_bs = child->data().bounding_sphere;
            if ( !child_bs.contains(bs, RTREE_BOUNDS_EPSILON) ) continue;
            RTreeNodeType* result = findLeafWithObject(child, loc, obj_id, bs);
            if (result != NULL) return result;
        }

        return NULL;
    }

protected:
    BoundingSphere bounding_sphere;
};


template<typename SimulationTraits>
class BoundingSphereData : public BoundingSphereDataBase<SimulationTraits, BoundingSphereData<SimulationTraits> > {
public:
    typedef BoundingSphereDataBase<SimulationTraits, BoundingSphereData<SimulationTraits> > ThisBase;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;

    BoundingSphereData()
     : BoundingSphereDataBase<SimulationTraits, BoundingSphereData>()
    {
    }

    BoundingSphereData(const LocationServiceCacheType* loc, const ObjectID& obj_id, const Time& t)
     : BoundingSphereDataBase<SimulationTraits, BoundingSphereData>( loc, obj_id, t )
    {
    }
};

/* Maintains the largest bounding sphere radius as well as the hierarchical bounding sphere.
 * We can cull if the largest bounding sphere, centered at the closest point on the
 * hierarchical bounding sphere, does not satisfy the constraints.
 */
template<typename SimulationTraits>
class MaxSphereData : public BoundingSphereDataBase<SimulationTraits, MaxSphereData<SimulationTraits> > {
public:
    typedef MaxSphereData NodeData; // For convenience/consistency
    typedef BoundingSphereDataBase<SimulationTraits, MaxSphereData<SimulationTraits> > ThisBase;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::TimeType Time;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;

    typedef Query<SimulationTraits> QueryType;

    MaxSphereData()
     : BoundingSphereDataBase<SimulationTraits, MaxSphereData>(),
       mMaxRadius(0.f)
    {
    }

    MaxSphereData(const LocationServiceCacheType* loc, const ObjectID& obj_id, const Time& t)
     : BoundingSphereDataBase<SimulationTraits, MaxSphereData>( loc, obj_id, t ),
       mMaxRadius( loc->worldBounds(obj_id, t).radius() )
    {
    }

    NodeData merge(const NodeData& other) const {
        NodeData result = BoundingSphereDataBase<SimulationTraits, MaxSphereData>::merge(other);
        result.mMaxRadius = std::max( mMaxRadius, other.mMaxRadius );
        return result;
    }

    // Merge the given info into this info
    void mergeIn(const NodeData& other) {
        BoundingSphereDataBase<SimulationTraits, MaxSphereData>::mergeIn(other);
        mMaxRadius = std::max( mMaxRadius, other.mMaxRadius );
    }

    // Check if this data satisfies the query constraints given
    bool satisfiesConstraints(const Vector3& qpos, const BoundingSphere& qbounds, const float qradius, const SolidAngle& qangle) {
        // We create a virtual stand in object which is the worst case object that could be in this subtree.
        // It's centered at the closest point on the hierarchical bounding sphere to the query, and has the
        // largest radius of any objects in the subtree.

        Vector3 obj_pos = ThisBase::bounding_sphere.center();
        float obj_radius = ThisBase::bounding_sphere.radius();

        // First, a special case is if the query is actually inside the hierarchical bounding sphere, in which
        // case the above description isn't accurate: in this case the worst position for the stand in object
        // is the exact location of the query.  So just let it pass.
        // FIXME we do this check manually for now, but BoundingSphere should provide it
        if (qbounds.radius() + obj_radius >= (qpos-obj_pos).length())
            return true;

        float standin_radius = mMaxRadius;

        Vector3 to_obj = obj_pos - qpos;
        to_obj = to_obj - to_obj.normal() * (obj_radius + qbounds.radius());

        // Must satisfy radius constraint
        if (qradius != QueryType::InfiniteRadius && to_obj.lengthSquared() > qradius*qradius)
            return false;

        // Must satisfy solid angle constraint
        SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, standin_radius);

        if (solid_angle >= qangle)
            return true;

        return false;
    }

    void verifyChild(const NodeData& child) const {
        BoundingSphereDataBase<SimulationTraits, MaxSphereData>::verifyChild(child);

        if ( child.mMaxRadius > mMaxRadius) {
            printf(
                "Child radius greater than recorded maximum child radius: %f > %f\n",
                child.mMaxRadius, mMaxRadius
            );
        }
    }
private:
    float mMaxRadius;
};

template<typename SimulationTraits, typename NodeData>
RTreeNode<SimulationTraits, NodeData>* RTree_choose_leaf(RTreeNode<SimulationTraits, NodeData>* root, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::ObjectIDType& obj_id, const typename SimulationTraits::TimeType& t) {
    typename SimulationTraits::BoundingSphereType obj_bounds = loc->worldBounds(obj_id, t);
    RTreeNode<SimulationTraits, NodeData>* node = root;

    while(!node->leaf()) {
        RTreeNode<SimulationTraits, NodeData>* min_increase_node = node->selectBestChildNode(loc, obj_id, t);
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
template<typename SimulationTraits, typename NodeData, typename ChildType, typename ChildOperations>
RTreeNode<SimulationTraits, NodeData>* RTree_split_node(RTreeNode<SimulationTraits, NodeData>* node, ChildType to_insert, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::TimeType& t) {
    ChildOperations child_ops;

    // collect the info for the children
    std::vector<ChildType> split_children;
    std::vector<NodeData> split_data;
    SplitGroups split_groups;

    // add all the children to the split vectors
    for(int i = 0; i < node->size(); i++) {
        split_children.push_back( child_ops.child(node, i) );
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
    RTreeNode<SimulationTraits, NodeData>* nn = new RTreeNode<SimulationTraits, NodeData>(node->capacity());
    nn->leaf(node->leaf());
    for(uint32 i = 0; i < split_children.size(); i++) {
        RTreeNode<SimulationTraits, NodeData>* newparent = (split_groups[i] == 0) ? node : nn;
        child_ops.insert( newparent, loc, split_children[i], t );
    }

    return nn;
}

// Fixes up the tree after insertion. Returns the new root node
template<typename SimulationTraits, typename NodeData>
RTreeNode<SimulationTraits, NodeData>* RTree_adjust_tree(RTreeNode<SimulationTraits, NodeData>* L, RTreeNode<SimulationTraits, NodeData>* LL, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::TimeType& t) {
    assert(L->leaf());
    RTreeNode<SimulationTraits, NodeData>* node = L;
    RTreeNode<SimulationTraits, NodeData>* nn = LL;

    while(true) { // loop until root, enter for root as well so we recompute its bounds
        RTreeNode<SimulationTraits, NodeData>* parent = node->parent();

        // FIXME this is inefficient
        node->recomputeData(loc, t);

        if (parent == NULL) break;

        RTreeNode<SimulationTraits, NodeData>* pp = NULL;
        if (nn != NULL) {
            if (parent->full())
                pp = RTree_split_node<SimulationTraits, NodeData, RTreeNode<SimulationTraits, NodeData>*, typename RTreeNode<SimulationTraits, NodeData>::NodeChildOperations>(parent, nn, loc, t);
            else
                parent->insert(nn);
        }

        node = parent;
        nn = pp;
    }

    // if we have a leftover split node, the root was split and we need to create
    // a new root one level higher
    if (nn != NULL) {
        RTreeNode<SimulationTraits, NodeData>* new_root = new RTreeNode<SimulationTraits, NodeData>(node->capacity());
        new_root->leaf(false);
        new_root->insert(node);
        new_root->insert(nn);

        node = new_root;
        nn = NULL;
    }

    return node;
}

// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename SimulationTraits, typename NodeData>
RTreeNode<SimulationTraits, NodeData>* RTree_insert_object(RTreeNode<SimulationTraits, NodeData>* root, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::ObjectIDType& obj_id, const typename SimulationTraits::TimeType& t) {
    RTreeNode<SimulationTraits, NodeData>* leaf_node = RTree_choose_leaf(root, loc, obj_id, t);

    RTreeNode<SimulationTraits, NodeData>* split_node = NULL;
    if (leaf_node->full())
        split_node = RTree_split_node<SimulationTraits, NodeData, typename SimulationTraits::ObjectIDType, typename RTreeNode<SimulationTraits, NodeData>::ObjectChildOperations>(leaf_node, obj_id, loc, t);
    else
        leaf_node->insert(loc, obj_id, t);

    RTreeNode<SimulationTraits, NodeData>* new_root = RTree_adjust_tree(leaf_node, split_node, loc, t);

    return new_root;
}

template<typename SimulationTraits, typename NodeData>
void RTree_verify_constraints(RTreeNode<SimulationTraits, NodeData>* root, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::TimeType& t) {
#ifdef PROXDEBUG
    for(int i = 0; i < root->size(); i++)
        root->data().verifyChild( root->childData(i, loc, t) );
    if (!root->leaf()) {
        for(int i = 0; i < root->size(); i++)
            RTree_verify_constraints(root->node(i), loc, t);
    }
#endif // def PROXDEBUG
}

/* Finds obj in the tree with the given root, assuming its at the position for time t. */
template<typename SimulationTraits, typename NodeData>
RTreeNode<SimulationTraits, NodeData>* RTree_find_leaf(RTreeNode<SimulationTraits, NodeData>* root, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::ObjectIDType& obj_id, const typename SimulationTraits::TimeType& t) {
    if (root == NULL) return NULL;
    return root->findLeafWithObject(loc, obj_id, t);
}

/* Takes a leaf node from which an object has been removed and, if it contains too few nodes,
 *  redistributes the objects it contains and removes the node from the tree.
 *  Returns the new root (which it may create because it might have to reinsert objects, which
 *  can itself cause a new root to appear.
 */
template<typename SimulationTraits, typename NodeData>
RTreeNode<SimulationTraits, NodeData>* RTree_condense_tree(RTreeNode<SimulationTraits, NodeData>* leaf, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::TimeType& t) {
    RTreeNode<SimulationTraits, NodeData>* n = leaf;
    std::queue<RTreeNode<SimulationTraits, NodeData>*> removedNodes;

    while(n->parent() != NULL) {
        RTreeNode<SimulationTraits, NodeData>* parent = n->parent();
        if (n->size() < 1) { // FIXME should be some larger value
            parent->erase(n);
            removedNodes.push(n);
        }
        else {
            n->recomputeData(loc, t);
        }
        n = parent;
    }

    RTreeNode<SimulationTraits, NodeData>* root = n;
    // There's a chance that the root node ended up with no elements, in which case it should be marked as a leaf node
    if (root->size() == 0)
        root->leaf(true);

    // FIXME this could reinsert entire nodes instead of individual objects, but we'd need a better idea of how to actually accomplish that...
    while(!removedNodes.empty()) {
        RTreeNode<SimulationTraits, NodeData>* removed = removedNodes.front();
        removedNodes.pop();
        if (removed->leaf()) {
            for(uint8 idx = 0; idx < removed->size(); idx++)
                root = RTree_insert_object(root, loc, removed->object(idx), t);
        }
        else {
            for(uint8 idx = 0; idx < removed->size(); idx++)
                removedNodes.push(removed->node(idx));
        }
    }

    return root;
}

/* Deletes the object from the given tree.  Returns the new root. */
template<typename SimulationTraits, typename NodeData>
RTreeNode<SimulationTraits, NodeData>* RTree_delete_object(RTreeNode<SimulationTraits, NodeData>* root, const LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::ObjectIDType& obj_id, const typename SimulationTraits::TimeType& t) {
    RTreeNode<SimulationTraits, NodeData>* leaf_with_obj = RTree_find_leaf(root, loc, obj_id, t);
    if (leaf_with_obj == NULL) {
        return root;
    }

    leaf_with_obj->erase(obj_id);
    RTreeNode<SimulationTraits, NodeData>* new_root = RTree_condense_tree(leaf_with_obj, loc, t);

    // We might need to shorten the tree if the root is left with only one child.
    if (!root->leaf() && root->size() == 1) {
        new_root = root->node(0);
        new_root->parent(NULL);
        root->clear();
        delete root;
    }
    return new_root;
}

} // namespace Prox
