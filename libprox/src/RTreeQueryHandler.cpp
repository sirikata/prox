/*  libprox
 *  RTreeQueryHandler.cpp
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

#include <prox/RTreeQueryHandler.hpp>
#include <prox/BoundingSphere.hpp>
#include <cassert>
#include <float.h>

#define RTREE_BOUNDS_EPSILON 0.1f // FIXME how should we choose this epsilon?

namespace Prox {

template<typename NodeData>
struct RTreeNode {
private:
    static const uint8 LeafFlag = 0x02; // elements are object pointers instead of node pointers

    union {
        RTreeNode** nodes;
        Object** objects;
        void** magic;
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

        NodeData data(RTreeNode* child, const Time& ) {
            return child->data();
        }

        void insert(RTreeNode* parent, RTreeNode* newchild, const Time& ) {
            parent->insert(newchild);
        }
    };

    struct ObjectChildOperations {
        Object* child(RTreeNode* parent, int idx) {
            return parent->object(idx);
        }

        NodeData data(Object* child, const Time& t) {
            return NodeData(child, t);
        }

        void insert(RTreeNode* parent, Object* newchild, const Time& t) {
            parent->insert(newchild,t);
        }
    };


    RTreeNode(uint8 _max_elements)
     : mParent(NULL), mData(), flags(0), count(0), max_elements(_max_elements)
    {
        elements.magic = new void*[max_elements];
        for(int i = 0; i < max_elements; i++)
            elements.magic[i] = NULL;

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

    Object* object(int i) const {
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

    NodeData childData(int i, const Time& t) {
        if (leaf())
            return NodeData(object(i), t);
        else
            return node(i)->data();
    }

    void recomputeData(const Time& t) {
        mData = NodeData();
        for(int i = 0; i < size(); i++)
            mData.mergeIn( childData(i, t) );
    }

    void clear() {
        count = 0;
        for(int i = 0; i < max_elements; i++)
            elements.magic[i] = NULL;
        mData = NodeData();
    }

    void insert(Object* obj, const Time& t) {
        assert (count < max_elements);
        assert (leaf() == true);
        elements.objects[count] = obj;
        count++;
        mData.mergeIn( NodeData(obj, t) );
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
    void erase(const Object* obj) {
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

    bool contains(const Object* obj) const {
        for(uint8 obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx] == obj) return true;
        return false;
    }

    RTreeNode* selectBestChildNode(const Object* obj, const Time& t) {
        return NodeData::selectBestChildNode(this, obj, t);
    }

    RTreeNode* findLeafWithObject(const Object* obj, const Time& t) {
        return NodeData::findLeafWithObject(this, obj, t);
    }
};


static const int32 UnassignedGroup = -1;
//typedef std::vector<NodeData> SplitData;
typedef std::vector<int32> SplitGroups;


struct RTreeBoundingInfo {
    BoundingSphere3f bounding_sphere;

    RTreeBoundingInfo()
     : bounding_sphere()
    {
    }

    RTreeBoundingInfo(const Object* obj, const Time& t)
     : bounding_sphere( obj->worldBounds(t) )
    {
    }

    // Return the result of merging this info with the given info
    RTreeBoundingInfo merge(const RTreeBoundingInfo& other) const {
        RTreeBoundingInfo result;
        result.bounding_sphere = bounding_sphere.merge(other.bounding_sphere);
        return result;
    }

    // Merge the given info into this info
    void mergeIn(const RTreeBoundingInfo& other) {
        bounding_sphere.mergeIn(other.bounding_sphere);
    }

    // Check if this data satisfies the query constraints given
    bool satisfiesConstraints(const Vector3f& qpos, const float qradius, const SolidAngle& qangle) {
        Vector3f obj_pos = bounding_sphere.center();
        Vector3f to_obj = obj_pos - qpos;

        // Must satisfy radius constraint
        if (qradius != Query::InfiniteRadius && (to_obj).lengthSquared() < (qradius+bounding_sphere.radius())*(qradius+bounding_sphere.radius()))
            return false;

        // Must satisfy solid angle constraint
        SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, bounding_sphere.radius());

        if (solid_angle < qangle)
            return false;

        return true;
    }

    // Given an object and a time, select the best child node to put the object in
    static RTreeNode<RTreeBoundingInfo>* selectBestChildNode(const RTreeNode<RTreeBoundingInfo>* node, const Object* obj, const Time& t) {
        float min_increase = 0.f;
        RTreeNode<RTreeBoundingInfo>* min_increase_node = NULL;

        BoundingSphere3f obj_bounds = obj->worldBounds(t);

        for(int i = 0; i < node->size(); i++) {
            RTreeNode<RTreeBoundingInfo>* child_node = node->node(i);
            BoundingSphere3f merged = child_node->data().bounding_sphere.merge(obj_bounds);
            float increase = merged.volume() - child_node->data().bounding_sphere.volume();
            if (min_increase_node == NULL || increase < min_increase) {
                min_increase = increase;
                min_increase_node = child_node;
            }
        }

        return min_increase_node;
    }

private:
    static RTreeNode<RTreeBoundingInfo>* findLeafWithObject(RTreeNode<RTreeBoundingInfo>* node, const Object* obj, const BoundingSphere3f& bs) {
        // For leaf nodes, simply check against all child objects
        if (node->leaf())
            return (node->contains(obj) ? node : NULL);

        // For internal nodes, check against child bounds, and then recursively check child
        for(uint8 child_idx = 0; child_idx < node->size(); child_idx++) {
            RTreeNode<RTreeBoundingInfo>* child = node->node(child_idx);
            BoundingSphere3f child_bs = child->data().bounding_sphere;
            if ( !child_bs.contains(bs, RTREE_BOUNDS_EPSILON) ) continue;
            RTreeNode<RTreeBoundingInfo>* result = findLeafWithObject(child, obj, bs);
            if (result != NULL) return result;
        }

        return NULL;
    }

public:
    // Given a root node and object and a time, find the leaf node which contains that object
    static RTreeNode<RTreeBoundingInfo>* findLeafWithObject(RTreeNode<RTreeBoundingInfo>* node, const Object* obj, const Time& t) {
        BoundingSphere3f bs = obj->worldBounds(t);
        return findLeafWithObject(node, obj, bs);
    }

    // Given a list of child data, choose two seeds for the splitting process in quadratic time
    static void pickSeedsQuadratic(const std::vector<RTreeBoundingInfo>& split_data, int32* seed0, int32* seed1) {
        *seed0 = -1; *seed1 = -1;
        float max_waste = -FLT_MAX;
        for(uint32 idx0 = 0; idx0 < split_data.size(); idx0++) {
            for(uint32 idx1 = idx0+1; idx1 < split_data.size(); idx1++) {
                BoundingSphere3f merged = split_data[idx0].bounding_sphere.merge(split_data[idx1].bounding_sphere);

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
    static void pickNextChild(std::vector<RTreeBoundingInfo>& split_data, const SplitGroups& split_groups, const RTreeBoundingInfo& group_data_0, const RTreeBoundingInfo& group_data_1, int32* next_child, int32* selected_group) {
        float max_preference = -1.0f;
        *next_child = -1;
        *selected_group = -1;

        for(uint32 i = 0; i < split_data.size(); i++) {
            if (split_groups[i] != UnassignedGroup) continue;

            BoundingSphere3f merged0 = group_data_0.bounding_sphere.merge(split_data[i].bounding_sphere);
            BoundingSphere3f merged1 = group_data_1.bounding_sphere.merge(split_data[i].bounding_sphere);

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

    void verifyChild(const RTreeBoundingInfo& child) const {
        if (! bounding_sphere.contains( child.bounding_sphere, RTREE_BOUNDS_EPSILON )) {
            printf("child exceeds bounds %f\n",
                bounding_sphere.radius() - ((bounding_sphere.center() - child.bounding_sphere.center()).length() + child.bounding_sphere.radius())
            );
        }
    }
};




template<typename NodeData>
RTreeNode<NodeData>* RTree_choose_leaf(RTreeNode<NodeData>* root, Object* obj, const Time& t) {
    BoundingSphere3f obj_bounds = obj->worldBounds(t);
    RTreeNode<NodeData>* node = root;

    while(!node->leaf()) {
        RTreeNode<NodeData>* min_increase_node = node->selectBestChildNode(obj, t);
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
template<typename NodeData, typename ChildType, typename ChildOperations>
RTreeNode<NodeData>* RTree_split_node(RTreeNode<NodeData>* node, ChildType* to_insert, const Time& t) {
    ChildOperations child_ops;

    // collect the info for the children
    std::vector<ChildType*> split_children;
    std::vector<NodeData> split_data;
    SplitGroups split_groups;

    // add all the children to the split vectors
    for(int i = 0; i < node->size(); i++) {
        split_children.push_back( child_ops.child(node, i) );
        split_data.push_back( node->childData(i,t) );
        split_groups.push_back(UnassignedGroup);
    }
    split_children.push_back( to_insert );
    split_data.push_back( child_ops.data(to_insert, t) );
    split_groups.push_back(UnassignedGroup);

    // find the initial seeds
    NodeData group_data_0, group_data_1;
    RTree_quadratic_pick_seeds(split_data, split_groups, group_data_0, group_data_1);

    // group the remaining ones
    for(uint32 i = 0; i < split_children.size()-2; i++)
        RTree_pick_next_child(split_data, split_groups, group_data_0, group_data_1);

    // copy data into the correct nodes
    node->clear();
    RTreeNode<NodeData>* nn = new RTreeNode<NodeData>(node->capacity());
    nn->leaf(node->leaf());
    for(uint32 i = 0; i < split_children.size(); i++) {
        RTreeNode<NodeData>* newparent = (split_groups[i] == 0) ? node : nn;
        child_ops.insert( newparent, split_children[i], t );
    }

    return nn;
}

// Fixes up the tree after insertion. Returns the new root node
template<typename NodeData>
RTreeNode<NodeData>* RTree_adjust_tree(RTreeNode<NodeData>* L, RTreeNode<NodeData>* LL, const Time& t) {
    assert(L->leaf());
    RTreeNode<NodeData>* node = L;
    RTreeNode<NodeData>* nn = LL;

    while(true) { // loop until root, enter for root as well so we recompute its bounds
        RTreeNode<NodeData>* parent = node->parent();

        // FIXME this is inefficient
        node->recomputeData(t);

        if (parent == NULL) break;

        RTreeNode<NodeData>* pp = NULL;
        if (nn != NULL) {
            if (parent->full())
                pp = RTree_split_node<NodeData, RTreeNode<NodeData>, typename RTreeNode<NodeData>::NodeChildOperations>(parent, nn, t);
            else
                parent->insert(nn);
        }

        node = parent;
        nn = pp;
    }

    // if we have a leftover split node, the root was split and we need to create
    // a new root one level higher
    if (nn != NULL) {
        RTreeNode<NodeData>* new_root = new RTreeNode<NodeData>(node->capacity());
        new_root->leaf(false);
        new_root->insert(node);
        new_root->insert(nn);

        node = new_root;
        nn = NULL;
    }

    return node;
}

// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename NodeData>
RTreeNode<NodeData>* RTree_insert_object(RTreeNode<NodeData>* root, Object* obj, const Time& t) {
    RTreeNode<NodeData>* leaf_node = RTree_choose_leaf(root, obj, t);

    RTreeNode<NodeData>* split_node = NULL;
    if (leaf_node->full())
        split_node = RTree_split_node<NodeData, Object, typename RTreeNode<NodeData>::ObjectChildOperations>(leaf_node, obj, t);
    else
        leaf_node->insert(obj, t);

    RTreeNode<NodeData>* new_root = RTree_adjust_tree(leaf_node, split_node, t);

    return new_root;
}

template<typename NodeData>
void RTree_verify_constraints(RTreeNode<NodeData>* root, const Time& t) {
#ifdef PROXDEBUG
    for(int i = 0; i < root->size(); i++)
        root->data().verifyChild( root->childData(i, t) );
    if (!root->leaf()) {
        for(int i = 0; i < root->size(); i++)
            RTree_verify_constraints(root->node(i), t);
    }
#endif // def PROXDEBUG
}

/* Finds obj in the tree with the given root, assuming its at the position for time t. */
template<typename NodeData>
RTreeNode<NodeData>* RTree_find_leaf(RTreeNode<NodeData>* root, const Object* obj, const Time& t) {
    if (root == NULL) return NULL;
    return root->findLeafWithObject(obj, t);
}

/* Takes a leaf node from which an object has been removed and, if it contains too few nodes,
 *  redistributes the objects it contains and removes the node from the tree.
 *  Returns the new root (which it may create because it might have to reinsert objects, which
 *  can itself cause a new root to appear.
 */
template<typename NodeData>
RTreeNode<NodeData>* RTree_condense_tree(RTreeNode<NodeData>* leaf, const Time& t) {
    RTreeNode<NodeData>* n = leaf;
    std::queue<RTreeNode<NodeData>*> removedNodes;

    while(n->parent() != NULL) {
        RTreeNode<NodeData>* parent = n->parent();
        if (n->size() < 1) { // FIXME should be some larger value
            parent->erase(n);
            removedNodes.push(n);
        }
        else {
            n->recomputeData(t);
        }
        n = parent;
    }

    RTreeNode<NodeData>* root = n;
    // There's a chance that the root node ended up with no elements, in which case it should be marked as a leaf node
    if (root->size() == 0)
        root->leaf(true);

    // FIXME this could reinsert entire nodes instead of individual objects, but we'd need a better idea of how to actually accomplish that...
    while(!removedNodes.empty()) {
        RTreeNode<NodeData>* removed = removedNodes.front();
        removedNodes.pop();
        if (removed->leaf()) {
            for(uint8 idx = 0; idx < removed->size(); idx++)
                root = RTree_insert_object(root, removed->object(idx), t);
        }
        else {
            for(uint8 idx = 0; idx < removed->size(); idx++)
                removedNodes.push(removed->node(idx));
        }
    }

    return root;
}

/* Deletes the object from the given tree.  Returns the new root. */
template<typename NodeData>
RTreeNode<NodeData>* RTree_delete_object(RTreeNode<NodeData>* root, const Object* obj, const Time& t) {
    RTreeNode<NodeData>* leaf_with_obj = RTree_find_leaf(root, obj, t);
    if (leaf_with_obj == NULL) {
        return root;
    }

    leaf_with_obj->erase(obj);
    RTreeNode<NodeData>* new_root = RTree_condense_tree(leaf_with_obj, t);

    // We might need to shorten the tree if the root is left with only one child.
    if (!root->leaf() && root->size() == 1) {
        new_root = root->node(0);
        new_root->parent(NULL);
        root->clear();
        delete root;
    }
    return new_root;
}




RTreeQueryHandler::RTreeQueryHandler(uint8 elements_per_node)
 : QueryHandler(),
   ObjectChangeListener(),
   QueryChangeListener(),
   mLastTime(0)
{
    mRTreeRoot = new RTree(elements_per_node);
}

RTreeQueryHandler::~RTreeQueryHandler() {
    mObjects.clear();
    for(QueryMap::iterator it = mQueries.begin(); it != mQueries.end(); it++) {
        QueryState* state = it->second;
        delete state;
    }
    mQueries.clear();
}

void RTreeQueryHandler::registerObject(Object* obj) {
    insert(obj, mLastTime);
    mObjects.insert(obj);
    obj->addChangeListener(this);
}

void RTreeQueryHandler::registerQuery(Query* query) {
    QueryState* state = new QueryState;
    mQueries[query] = state;
    query->addChangeListener(this);
}

void RTreeQueryHandler::tick(const Time& t) {
    // FIXME we should have a better way of updating instead of delete + insert
    for(ObjectSet::iterator obj_it = mObjects.begin(); obj_it != mObjects.end(); obj_it++) {
        deleteObj(*obj_it, mLastTime);
    }
    for(ObjectSet::iterator obj_it = mObjects.begin(); obj_it != mObjects.end(); obj_it++) {
        insert(*obj_it, t);
    }

    RTree_verify_constraints(mRTreeRoot, t);
    int count = 0;
    int ncount = 0;
    for(QueryMap::iterator query_it = mQueries.begin(); query_it != mQueries.end(); query_it++) {
        Query* query = query_it->first;
        QueryState* state = query_it->second;
        QueryCache newcache;

        Vector3f qpos = query->position(t);
        float qradius = query->radius();
        const SolidAngle& qangle = query->angle();

        std::stack<RTree*> node_stack;
        node_stack.push(mRTreeRoot);
        while(!node_stack.empty()) {
            RTree* node = node_stack.top();
            node_stack.pop();

            if (node->leaf()) {
                for(int i = 0; i < node->size(); i++) {
                    count++;
                    if (node->childData(i,t).satisfiesConstraints(qpos, qradius, qangle))
                        newcache.add(node->object(i)->id());
                }
            }
            else {
                for(int i = 0; i < node->size(); i++) {
                    count++;
                    if (node->childData(i,t).satisfiesConstraints(qpos, qradius, qangle))
                        node_stack.push(node->node(i));
                    else
                        ncount++;
                }
            }
        }

        std::deque<QueryEvent> events;
        state->cache.exchange(newcache, &events);

        query->pushEvents(events);
    }
    printf("count: %d %d\n", count, ncount);
    mLastTime = t;
}

void RTreeQueryHandler::objectPositionUpdated(Object* obj, const MotionVector3f& old_pos, const MotionVector3f& new_pos) {
    // FIXME should use more efficient update approach
    deleteObj(obj, mLastTime);
    insert(obj, mLastTime);
}

void RTreeQueryHandler::objectBoundingSphereUpdated(Object* obj, const BoundingSphere3f& old_bounds, const BoundingSphere3f& new_bounds) {
    // FIXME should use more efficient update approach
    deleteObj(obj, mLastTime);
    insert(obj, mLastTime);
}

void RTreeQueryHandler::objectDeleted(const Object* obj) {
    assert( mObjects.find(const_cast<Object*>(obj)) != mObjects.end() );
    mObjects.erase(const_cast<Object*>(obj));
    deleteObj(obj, mLastTime);
}

void RTreeQueryHandler::queryPositionUpdated(Query* query, const MotionVector3f& old_pos, const MotionVector3f& new_pos) {
    // Nothing to be done, we use values directly from the query
}

void RTreeQueryHandler::queryDeleted(const Query* query) {
    QueryMap::iterator it = mQueries.find(const_cast<Query*>(query));
    assert( it != mQueries.end() );
    QueryState* state = it->second;
    delete state;
    mQueries.erase(it);
}

void RTreeQueryHandler::insert(Object* obj, const Time& t) {
    mRTreeRoot = RTree_insert_object(mRTreeRoot, obj, t);
}

void RTreeQueryHandler::deleteObj(const Object* obj, const Time& t) {
    mRTreeRoot = RTree_delete_object(mRTreeRoot, obj, t);
}


} // namespace Prox
