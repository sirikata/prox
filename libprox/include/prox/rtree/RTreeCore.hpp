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

#ifndef _PROX_RTREE_CORE_HPP_
#define _PROX_RTREE_CORE_HPP_

#include <prox/util/Platform.hpp>
#include <prox/base/LocationServiceCache.hpp>
#include <prox/rtree/Constraints.hpp>
#include <prox/base/AggregateListener.hpp>
#include <float.h>

#define RTREE_BOUNDS_EPSILON 0.1f // FIXME how should we choose this epsilon?

namespace Prox {

template<typename CutNode>
struct CutNodeContainer {
    typedef typename CutNode::CutType Cut;
    typedef Cut* CutPtr;
    typedef std::tr1::unordered_map<Cut*, CutNode*> CutNodeList;
    CutNodeList cuts;

    typedef typename CutNodeList::iterator CutNodeListIterator;
    typedef typename CutNodeList::const_iterator CutNodeListConstIterator;
    CutNodeListIterator cutNodesBegin() { return cuts.begin(); }
    CutNodeListConstIterator cutNodesBegin() const { return cuts.begin(); }
    CutNodeListIterator cutNodesEnd() { return cuts.end(); }
    CutNodeListConstIterator cutNodesEnd() const { return cuts.end(); }

    void insertCutNode(CutNode* cn) {
        cuts[cn->parent] = cn;
    }
    void eraseCutNode(CutNode* cn) {
        assert(cuts.find(cn->parent) != cuts.end());
        assert(cuts[cn->parent] == cn);
        cuts.erase(cn->parent);
    }
    size_t cutNodesSize() const {
        return cuts.size();
    }
    bool cutNodesEmpty() const {
        return cuts.empty();
    }
    CutNodeListIterator findCutNode(CutPtr const& ct) {
        return cuts.find(ct);
    }
    CutNodeListIterator findCutNode(CutPtr const& ct) const {
        return cuts.find(ct);
    }
    CutNodeListIterator findCutNode(const CutNode* cn) {
        CutNodeListIterator it = cuts.find(cn->parent);
        assert(it == cuts.end() || it->second == cn);
        return it;
    }
    CutNodeListConstIterator findCutNode(const CutNode* cn) const {
        CutNodeListConstIterator it = cuts.find(cn->parent);
        assert(it == cuts.end() || it->second == cn);
        return it;
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
    typedef typename SimulationTraits::ObjectIDNullType ObjectIDNull;
    typedef typename SimulationTraits::ObjectIDRandomType ObjectIDRandom;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef Aggregator<SimulationTraits> AggregatorType;
    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef RTreeLeafNode<SimulationTraits, CutNode> LeafNode;

    typedef typename CutNode::RangeType CutNodeRange;
    typedef typename CutNode::CutType Cut;
    typedef std::vector<typename CutNode::RangeType> CutRangeVector;
    typedef std::map<typename CutNode::CutType*, CutRangeVector> CutRangeMap;

    typedef std::tr1::function<void(const LocCacheIterator&, RTreeNode*)> ObjectLeafChangedCallback;
    typedef std::tr1::function<RTreeNode*(const ObjectID&)> GetObjectLeafCallback;

    // The root was left with only one child node, cuts on the old root need to
    // "refine" themselves to the new node by simple replacement.  Parameters
    // are the old root followed by the new root.
    typedef std::tr1::function<void(CutNode*, RTreeNode*, RTreeNode*)> RootReplacedByChildCallback;
    // CutNode that split occurred at, the node that was split, the new node.
    typedef std::tr1::function<void(CutNode*, RTreeNode*, RTreeNode*)> NodeSplitCallback;
    typedef std::tr1::function<void(CutNode*, RTreeNode*)> LiftCutCallback;
    // Reorder the cut by reorganizing the ranges in the vector into the order
    // they appear in the vector. They'll be adjacent, so the remainder of the
    // cut on the left and right of these ranges should remain as is.
    typedef std::tr1::function<void(typename CutNode::CutType*, const CutRangeVector&)> ReorderCutCallback;
    typedef std::tr1::function<void(CutNode*, const LocCacheIterator&, int)> ObjectInsertedCallback;
    typedef std::tr1::function<void(CutNode*, const LocCacheIterator&, bool permanent)> ObjectRemovedCallback;

    typedef uint16 Index;

    struct Callbacks {
        AggregatorType* aggregator;
        AggregateListenerType* aggregate;
        ObjectLeafChangedCallback objectLeafChanged;
        GetObjectLeafCallback getObjectLeaf;
        RootReplacedByChildCallback rootReplaced;
        NodeSplitCallback nodeSplit;
        LiftCutCallback liftCut;
        ReorderCutCallback reorderCut;
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

        NodeData data(LocationServiceCacheType* loc, const LocCacheIterator& child, const Time& t) {
            return NodeData(loc, child, t);
        }

        void insert(RTreeNode* parent, LocationServiceCacheType* loc, const LocCacheIterator& newchild, const Time& t, const Callbacks& cb) {
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

        if (callbacks.aggregate != NULL) callbacks.aggregate->aggregateCreated(callbacks.aggregator, aggregate);
    }

    // This version initializes the node from known data, i.e. for
    // replicating existing trees.
    RTreeNode(Index _max_elements, LocationServiceCacheType* loc, const LocCacheIterator& node, const Time& t, const Callbacks& callbacks)
     : mParent(NULL), mData(loc, node, t),
       flags(0), count(0),
       max_elements(_max_elements),
       aggregate( loc->iteratorID(node) )
    {
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(LeafNode) );
        uint32 magic_size = max_element_size * max_elements;
        elements.magic = new uint8[magic_size];
        memset(elements.magic, 0, magic_size);

        leaf(true);

        // FIXME(ewencp) Better initialization of mData? Included max object
        // size but overwrite it for aggregates?

        if (callbacks.aggregate != NULL) callbacks.aggregate->aggregateCreated(callbacks.aggregator, aggregate);
    }

    // We have a destroy method and hide the destructor in private in order to
    // ensure the aggregate callbacks get invoked properly.
    void destroy(LocationServiceCacheType* loc, const Callbacks& callbacks) {
        // Make sure we get all the destructors right by just clearing the
        // entire node
        clear(loc, callbacks);

        if (callbacks.aggregate != NULL) callbacks.aggregate->aggregateDestroyed(callbacks.aggregator, aggregate);
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

    // Size of this tree, including this node.
    int treeSize() const {
        int result = 1;
        // For leaves, we count the children ourselves
        if (leaf())
            result += size();
        else // Otherwise recurse
            for(Index i = 0; i < size(); i++)
                result += node(i)->treeSize();
        return result;
    }

    const ObjectID& aggregateID() const { return aggregate; }

    RTreeNode* parent() const {
        return mParent;
    }
    void parent(RTreeNode* _p) {
        mParent = _p;
    }
    const ObjectID& parentAggregateID() const {
        if (mParent != NULL)
            return mParent->aggregateID();
        static ObjectID no_parent_id = ObjectIDNull()();
        return no_parent_id;
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

    NodeData childData(int i, LocationServiceCacheType* loc, const Time& t) {
        if (leaf())
            return NodeData(loc, object(i).object, t);
        else
            return node(i)->data();
    }

    void recomputeData(LocationServiceCacheType* loc, const Time& t, const Callbacks& cb) {
        BoundingSphere orig = mData.getBounds();
        mData = NodeData();
        for(int i = 0; i < size(); i++)
            mData.mergeIn( childData(i, loc, t) );
        BoundingSphere updated = mData.getBounds();
        if (cb.aggregate != NULL && updated != orig)
            cb.aggregate->aggregateBoundsUpdated(cb.aggregator, aggregate, mData.getBounds());
    }

private:
    void notifyRemoved(LocationServiceCacheType* loc, const LocCacheIterator& obj, const Callbacks& cb, bool permanent) {
        ObjectID obj_id = loc->iteratorID(obj);
        if (cb.objectRemoved) {
            for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = this->cutNodesBegin(); cut_it != this->cutNodesEnd(); cut_it++)
                cb.objectRemoved(cut_it->second, obj, permanent);
        }

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildRemoved(cb.aggregator, aggregate, obj_id, mData.getBounds());
    }

public:

    void clear(LocationServiceCacheType* loc, const Callbacks& cb) {
        Index old_count = count;

        // If we have child objects, we need to notify cuts of removal
        // To allow validation after notifyRemoved we need to work 1 at a time.
        if (leaf()) {
            for(int i = 0; i < old_count; i++) {
                count = old_count-i-1;
                notifyRemoved(loc, this->elements.objects[old_count-i-1].object, cb, false);
                // Explicitly call destructor, necessary since we use placement
                // new to handle constructing objects in place (see insert())
                this->elements.objects[old_count-i-1].~LeafNode();
            }
        }
        else {
            // If we're removing child nodes, we don't need to notify
            // of object removal, but we do need to notify of
            // aggregate child removal
            for(Index i = 0; i < count; i++)
                if (cb.aggregate != NULL) cb.aggregate->aggregateChildRemoved(cb.aggregator, aggregate, node(i)->aggregate, mData.getBounds());
            count = 0;
        }

#ifdef PROXDEBUG
        // Clear out data for safety
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(LeafNode) );
        for(uint32 i = 0; i < max_element_size * max_elements; i++)
            elements.magic[i] = 0;
#endif

        mData = NodeData();
    }

    void insert(LocationServiceCacheType* loc, const LocCacheIterator& obj, const Time& t, const Callbacks& cb) {
        assert (count < max_elements);
        assert (leaf() == true);

        int idx = count;
        // Use placement new to get the constructor called without using
        // assignment (which would result in a destructor on existing (likely
        // bogus or leftover) data being called. Careful destruction handled in
        // erase() and clear.
        new (&(elements.objects[idx])) LeafNode(obj);
        cb.objectLeafChanged(obj, this);
        count++;
        mData.mergeIn( NodeData(loc, obj, t) );

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildAdded(cb.aggregator, aggregate, loc->iteratorID(obj), mData.getBounds());

        if (cb.objectInserted) {
            for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = this->cutNodesBegin(); cut_it != this->cutNodesEnd(); cut_it++)
                cb.objectInserted(cut_it->second, obj, idx);
        }
    }

    /** Insert node as a child and invoke callbacks if necessary. If
     *  provided, after indicates where in the list the node should
     *  appear. The order of existing items are preserved (everything
     *  is shifted rather than using swaps); this operation is safe
     *  for cuts as long as you take care of handling the inserted
     *  node. If not specified, the node is added at the end of the
     *  current list.
     */
    void insert(RTreeNode* node, const Callbacks& cb, RTreeNode* after = NULL) {
        assert (count < max_elements);
        assert (leaf() == false);
        assert (after == NULL || (after->parent() == this && contains(after)));

        node->parent(this);

        // Figure out the index where this item will appear
        Index idx = (after == NULL ? count : (indexOf(after)+1));
        // Shift items out of the way. Be careful with the loop
        // variables and conditions since we need to avoid negative
        // numbers since Index is unsigned
        for(Index shift_idx = count; shift_idx > idx; shift_idx--)
            elements.nodes[shift_idx] = elements.nodes[shift_idx-1];
        // And insert the node and update stats
        elements.nodes[idx] = node;
        count++;
        mData.mergeIn( node->data() );

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildAdded(cb.aggregator, aggregate, node->aggregate, mData.getBounds());
    }

    // NOTE: does not recalculate the bounding sphere
    void erase(LocationServiceCacheType* loc, const LocCacheIterator& obj, bool temporary, const Callbacks& cb) {
        assert(count > 0);
        assert(leaf() == true);

        // find obj
        Index obj_idx;
        for(obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx].object == obj) break;
        // push all the other objects back one. NOTE: Unlike clear, we don't
        // need to to call the destructor on the removed element because all
        // these = operators will destroy the objects that were in their place.
        for(Index rem_idx = obj_idx; rem_idx < count-1; rem_idx++)
            elements.objects[rem_idx] = elements.objects[rem_idx+1];
        // Instead, we need to call it on the *last* element, because we shifted
        // everything down and would otherwise leave the last one in its place.
        // So we explicitly call destructor. See placement new in insert.
        this->elements.objects[count-1].~LeafNode();
        // Finally, reduce the count
        count--;

        // This is only invoked due to the deletion of an individual object, so
        // we specify permanent deletion (last param)
        notifyRemoved(loc, obj, cb, !temporary);
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

        if (cb.aggregate != NULL) cb.aggregate->aggregateChildRemoved(cb.aggregator, aggregate, node->aggregate, mData.getBounds());
    }

    // Removes the last child in the node and returns the pointer to it.
    // NOTE: does not recalculate the bounding sphere.
    RTreeNode* erasePop(const Callbacks& cb) {
        assert(count > 0);
        assert(leaf() == false);
        int idx = size() - 1;
        RTreeNode* retval = node(idx);
        erase(idx);
        return retval;
    }

    bool contains(const LocCacheIterator& obj) const {
        assert(leaf());
        for(Index obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx] == obj) return true;
        return false;
    }
    bool contains(RTreeNode* node) const {
        assert(!leaf());
        for(Index idx = 0; idx < count; idx++)
            if (elements.nodes[idx] == node) return true;
        return false;
    }


    Index indexOf(const LocCacheIterator& obj) const {
        assert(contains(obj));
        assert(leaf());
        for(Index idx = 0; idx < count; idx++)
            if (elements.objects[idx] == obj) return idx;
        return -1;
    }
    Index indexOf(RTreeNode* node) const {
        assert(contains(node));
        assert(!leaf());
        for(Index idx = 0; idx < count; idx++)
            if (elements.nodes[idx] == node) return idx;
        return -1;
    }

    RTreeNode* selectBestChildNode(LocationServiceCacheType* loc, const LocCacheIterator& obj, const Time& t) {
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

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    BoundingSphereDataBase()
     : bounding_sphere(Vector3(0,0,0), -1.f) // Invalid, should merge properly
    {
    }

    BoundingSphereDataBase(LocationServiceCacheType* loc, const LocCacheIterator& obj, const Time& t)
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

        return (satisfiesConstraintsBounds<SimulationTraits>(obj_pos, obj_radius, qpos, qregion, qmaxsize, qangle, qradius) != -1);
    }
    // Get the score (or -1) for this data, given the query constraints
    float32 score(const Vector3& qpos, const BoundingSphere& qregion, const float qmaxsize, const SolidAngle& qangle, const float qradius) const {
        Vector3 obj_pos = bounding_sphere.center();
        float obj_radius = bounding_sphere.radius();

        return satisfiesConstraintsBounds<SimulationTraits>(obj_pos, obj_radius, qpos, qregion, qmaxsize, qangle, qradius);
    }

    // Given an object and a time, select the best child node to put the object in
    static RTreeNodeType* selectBestChildNode(const RTreeNodeType* node, LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
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

    /** Gets the volume of this bounds of this region. */
    float volume() const {
        return getBounds().volume();
    }

    float surfaceArea() const {
        return getBounds().surfaceArea();
    }

    /** Get the radius within which a querier asking for the given minimum solid
     *  angle will get this data as a result, i.e. the radius within which this
     *  node will satisfy the given query.
     */
    float getValidRadius(const SolidAngle& min_sa) const {
        // There's a minimum value based on when we end up *inside* the volume
        float bounds_max = getBounds().radius();
        // Otherwise, we just invert the solid angle formula
        float sa_max = min_sa.maxDistance(getBounds().radius());
        return std::max( bounds_max, sa_max );
    }

    static float hitProbability(const NodeData& parent, const NodeData& child) {
        static SolidAngle rep_sa(.01); // FIXME
        float parent_max_rad = parent.getValidRadius(rep_sa);
        float child_max_rad = child.getValidRadius(rep_sa);
        float ratio = child_max_rad / parent_max_rad;
        return ratio*ratio;
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

    BoundingSphereData(LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t)
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

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    MaxSphereData()
     : BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>(),
       mMaxRadius(0.f)
    {
    }

    MaxSphereData(LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t)
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

        return (satisfiesConstraintsBoundsAndMaxSize<SimulationTraits>(obj_pos, obj_radius, mMaxRadius, qpos, qregion, qmaxsize, qangle, qradius) != -1);
    }
    // Get the score (or -1) for this data, given the query constraints
    float32 score(const Vector3& qpos, const BoundingSphere& qregion, const float qmaxsize, const SolidAngle& qangle, const float qradius) const {
        Vector3 obj_pos = ThisBase::bounding_sphere.center();
        float obj_radius = ThisBase::bounding_sphere.radius();

        return satisfiesConstraintsBoundsAndMaxSize<SimulationTraits>(obj_pos, obj_radius, mMaxRadius, qpos, qregion, qmaxsize, qangle, qradius);
    }

    // Given an object and a time, select the best child node to put the object in
    static RTreeNodeType* selectBestChildNode(const RTreeNodeType* node, LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
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

    /** Gets the volume of this bounds of this region. */
    float volume() const {
        return getBounds().volume();
    }

    /** Get the radius within which a querier asking for the given minimum solid
     *  angle will get this data as a result, i.e. the radius within which this
     *  node will satisfy the given query.
     */
    float getValidRadius(const SolidAngle& min_sa) const {
        // There's a minimum value based on when we end up *inside* the volume
        float bounds_max = getBounds().radius() + mMaxRadius;
        // Otherwise, we just invert the solid angle formula
        float sa_max = min_sa.maxDistance(mMaxRadius) + getBounds().radius();
        return std::max( bounds_max, sa_max );
    }

    static float hitProbability(const NodeData& parent, const NodeData& child) {
        static SolidAngle rep_sa(.01); // FIXME
        float parent_max_rad = parent.getValidRadius(rep_sa);
        float child_max_rad = child.getValidRadius(rep_sa);
        float ratio = child_max_rad / parent_max_rad;
        return ratio*ratio;
    }

private:
    float mMaxRadius;
};

template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_choose_leaf(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
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

#ifdef LIBPROX_LIFT_CUTS

// Splits a node, inserting the given node, and returns the second new node
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_split_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    ChildType to_insert,
    LocationServiceCache<SimulationTraits>* loc,
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
        RTree_lift_cut_nodes_from_tree(node, parent, cb);
    else
        RTree_lift_cut_nodes_from_tree(node, node, cb);
    RTree_verify_no_cut_nodes_in_tree(node);


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
    node->clear(loc, cb);
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
            CutNode* cutnode = cut_it->second;
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
    LocationServiceCache<SimulationTraits>* loc,
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

#else // LIBPROX_LIFT_CUTS

// Splits a node, inserting the given node, and returns the second new node. In
// this version, we avoid dealing with cuts and leave that work up to other,
// follow-up methods. In other words, this *only* does the splitting up of
// children + allocation of new node.
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_split_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    ChildType to_insert,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

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
    node->clear(loc, cb);
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn = new RTreeNode<SimulationTraits, NodeData, CutNode>(node->capacity(), cb);
    nn->leaf(node->leaf());
    for(uint32 i = 0; i < split_children.size(); i++) {
        RTreeNode<SimulationTraits, NodeData, CutNode>* newparent = (split_groups[i] == 0) ? node : nn;
        child_ops.insert( newparent, loc, split_children[i], t, cb);
    }

    return nn;
}

// Extract segments of cuts under the given node. The output is a map of vectors
// of ranges so you can build up lists of ranges for each cut
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_extract_cut_segments(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    std::map<typename CutNode::CutType*, std::vector<typename CutNode::RangeType> >* segments_out
)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // A contiguous range within a cut. Used to record contiguous segments of a
    // cut that are being moved.
    typedef typename CutNode::RangeType CutNodeRange;
    typedef typename CutNode::CutType Cut;
    typedef std::vector<typename CutNode::RangeType> CutRangeVector;
    typedef std::map<typename CutNode::CutType*, CutRangeVector> CutRangeMap;

    // This assumes that the structure of the cut in the subtree is valid. A cut
    // must start and end on the left and right edges of the tree to be valid,
    // so we just need to traverse those edges. But since cuts may start and end
    // at different heights, we traverse looking for starting cut nodes, then
    // traverse the other side, looking for ending cut nodes.

    // The left (starting) edge
    RTreeNodeType* n = node;
    while(true) {
        // Add new entries for each cut node found here
        for(typename RTreeNodeType::CutNodeListConstIterator cut_it = n->cutNodesBegin(); cut_it != n->cutNodesEnd(); cut_it++) {
            CutNode* cutnode = cut_it->second;
            Cut* cut = cutnode->getParent();
            CutRangeVector& cut_ranges = (*segments_out)[cut];
            // If we have any items in here already, they should be completed
            assert(cut_ranges.empty() || (cut_ranges.back().first != NULL && cut_ranges.back().second != NULL));
            // Insert a new item
            cut_ranges.push_back( CutNodeRange(cutnode, NULL) );
        }

        if (n->leaf()) break;
        assert(!n->empty());
        n = n->node(0);
    }

    // The right (ending) edge
    n = node;
    while(true) {
        // Add new entries for each cut node found here
        for(typename RTreeNodeType::CutNodeListConstIterator cut_it = n->cutNodesBegin(); cut_it != n->cutNodesEnd(); cut_it++) {
            CutNode* cutnode = cut_it->second;
            Cut* cut = cutnode->getParent();
            CutRangeVector& cut_ranges = (*segments_out)[cut];
            // We better have a last entry that's half complete
            assert(!cut_ranges.empty() && (cut_ranges.back().first != NULL && cut_ranges.back().second == NULL));
            // Insert a new item
            cut_ranges.back().second = cutnode;
        }

        if (n->leaf()) break;
        assert(!n->empty());
        n = n->node( n->size()-1 );
    }
}

// Walks up the tree, dealing with cascading insertions due to an inserted
// object and recomputing aggregate info as necessary. Returns the new root node
// (or existing one if it remains the same)
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_adjust_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* L,
    RTreeNode<SimulationTraits, NodeData, CutNode>* LL,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
 {
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // A contiguous range within a cut. Used to record contiguous segments of a
    // cut that are being moved.
    typedef typename CutNode::RangeType CutNodeRange;
    typedef typename CutNode::CutType Cut;
    typedef std::vector<typename CutNode::RangeType> CutRangeVector;
    typedef std::map<typename CutNode::CutType*, CutRangeVector> CutRangeMap;

    assert(L->leaf());
    RTreeNode<SimulationTraits, NodeData, CutNode>* node = L;
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn = LL;

    while(true) { // loop until root, enter for root as well so we recompute its bounds
        RTreeNode<SimulationTraits, NodeData, CutNode>* parent = node->parent();

        // FIXME this is inefficient
        node->recomputeData(loc, t, cb);

        RTreeNode<SimulationTraits, NodeData, CutNode>* pp = NULL;
        if (nn != NULL) {
            if (cb.nodeSplit) {
                assert(cb.nodeSplit && cb.reorderCut);
                // If we saw a split, we need to deal with cleaning up
                // cuts. First, we need to get new cut nodes into cuts for the
                // newly generated node. We only need to insert these for cuts
                // going through the node that we split into two. Cuts that pass
                // through the tree lower down should have already been taken
                // care of and should alrady span the entire width of the tree.
                for(typename RTreeNodeType::CutNodeListConstIterator cut_it = node->cutNodesBegin(); cut_it != node->cutNodesEnd(); cut_it++) {
                    CutNode* cutnode = cut_it->second;
                    cb.nodeSplit(cutnode, node, nn);
                }

                // Now we should have all the parts of the cut that should be in
                // the tree. Now we need to patch up the order of things since
                // the split rearranged cuts. This rearrangement can't just
                // apply to cuts passing through these nodes: any cut lower in
                // the tree could have become jumbled. However, we only need to
                // deal with "blocks" of the cuts under each child since we only
                // jumble things around one layer down (the children).
                if (!node->leaf()) {
                    // Only need to do (and only can do) rearrangement if we're
                    // not at leaves.
                    assert(!nn->leaf());

                    // Extract a segment of the cut for each cut
                    CutRangeMap segments_out;
                    for(typename RTreeNodeType::Index ci = 0; ci < node->size(); ci++) {
                        int nsegments_before = segments_out.size();
                        RTree_extract_cut_segments(node->node(ci), &segments_out);
                        int nsegments_after = segments_out.size();
                        // Because we are only examining siblings, it shouldn't
                        // be possible for the number of cuts to change (aside
                        // from the initial 0 -> total number of cuts through
                        // subtree).
                        assert(nsegments_before == 0 || nsegments_before == nsegments_after);
                    }
                    for(typename RTreeNodeType::Index ci = 0; ci < nn->size(); ci++) {
                        int nsegments_before = segments_out.size();
                        RTree_extract_cut_segments(nn->node(ci), &segments_out);
                        int nsegments_after = segments_out.size();
                        // Because we are only examining siblings, it shouldn't
                        // be possible for the number of cuts to change (aside
                        // from the initial 0 -> total number of cuts through
                        // subtree).
                        assert(nsegments_before == 0 || nsegments_before == nsegments_after);
                    }

                    // And rearrange those parts of the cut. The order in the
                    // list of segments is the order we want them to now appear.
                    for(typename CutRangeMap::iterator cut_it = segments_out.begin(); cut_it != segments_out.end(); cut_it++) {
                        Cut* cut = cut_it->first;
                        CutRangeVector& range_vec = cut_it->second;
                        cb.reorderCut(cut, range_vec);
                    }
                }
            }

            // We need to deal with cuts whether we're at the root or
            // not, so we have to break out of the loop after we get
            // into this section where we process cuts and deal with
            // further splits. If we don't have a parent, we're going
            // to take care of generating a new root for the now two
            // roots later on (and there can't be any cut management
            // there since it's an entirely new layer).
            if (parent != NULL) {
                if (parent->full()) {
                    pp = RTree_split_node<SimulationTraits, NodeData, CutNode, RTreeNode<SimulationTraits, NodeData, CutNode>*, typename RTreeNode<SimulationTraits, NodeData, CutNode>::NodeChildOperations>(parent, nn, loc, t, cb);
                }
                else {
                    // We need to be careful about insertion. If the
                    // node we're splitting isn't the last node in the
                    // parent, we could screw up the order of the cut
                    // if we just insert it at the end since the
                    // nodeSplit callback just tells it that we're
                    // splitting that node, and that code assumes
                    // (since there's no other reasonable approach)
                    // that the new node is next to the old one.
                    //
                    // So, to handle this properly, we need to insert
                    // right after the node we split into two. We
                    // specify this to the insert call, which shifts
                    // the other elements over.
                    // insert(new_node=nn, callbacks=cb, optional after=node)
                    parent->insert(nn, cb, node);
                }
            }
        }

        if (parent == NULL) break;

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

#endif


// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_insert_object_at_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_node,
    LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{

    RTreeNode<SimulationTraits, NodeData, CutNode>* split_node = NULL;
    if (leaf_node->full()) {
        split_node = RTree_split_node<SimulationTraits, NodeData, CutNode, typename LocationServiceCache<SimulationTraits>::Iterator, typename RTreeNode<SimulationTraits, NodeData, CutNode>::ObjectChildOperations>(leaf_node, obj_id, loc, t, cb);
        // This leaves the new split_node without a parent and without any cuts
        // passing through it. adjust_tree will take care of cleaning that up as
        // it also deals with additional, cascading splits.
    }
    else {
        leaf_node->insert(loc, obj_id, t, cb);
    }

    RTreeNode<SimulationTraits, NodeData, CutNode>* new_root = RTree_adjust_tree(leaf_node, split_node, loc, t, cb);

    return new_root;
}

// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_insert_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    RTreeNodeType* leaf_node = RTree_choose_leaf(root, loc, obj_id, t);
    return RTree_insert_object_at_node(leaf_node, loc, obj_id, t, cb);
}



template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_create_new_node(
    uint16 elements_per_node,
    LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& node,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    return new RTreeNodeType(elements_per_node, loc, node, t, cb);
}



// Inserts a new, empty node into the tree with the given node as it's parent,
// updating any ancestors nodes as necessary. Returns the new node
// (unlike insert object, it should never be possible for this to
// result in a new root since it assumes space is available for the node).
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_insert_new_node_at_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* new_node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* parent_node,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // Sanity check the parent
    assert(!parent_node->full());
    // Should be either marked as not a leaf or should be empty so we can change it
    assert(!parent_node->leaf() || parent_node->empty());
    // And once we know it's safe, make sure we have the setting right
    parent_node->leaf(false);

    parent_node->insert(new_node, cb);

    // No cleanup -- cannot cause overflow and splitting
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
void RTree_verify_constraints(RTreeNode<SimulationTraits, NodeData, CutNode>* root, LocationServiceCache<SimulationTraits>* loc, const typename SimulationTraits::TimeType& t) {
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


/** Get all cuts in from_node to "lift" themselves up to to_node so the node can
 *  be operated on. This may require the cuts adjusting other subtrees as well,
 *  so the tree should be in a clean state before calling this.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_lift_cut_nodes(
    RTreeNode<SimulationTraits, NodeData, CutNode>* from_node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* to_node,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    // Only notify cuts if we're moving *above* this node, not if this node is
    // both the starting point for recursion and the destination node
    if (from_node != to_node) {
        // Notify cuts to lift up to the right node
        // NOTE: We use this approach since the callback likely adjusts the cut
        // node list in from_node
        while(!from_node->cutNodesEmpty())
            cb.liftCut(from_node->cutNodesBegin()->second, to_node);
    }
}

/** Get all cuts in from_node and its children to "lift" themselves up to
 *  to_node so the subtree can be operated on.  This may require the cuts
 *  adjusting other subtrees as well, so the tree should be in a clean state
 *  before calling this.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_lift_cut_nodes_from_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* from_node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* to_node,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename RTreeNodeType::Index Index;

    RTree_lift_cut_nodes(from_node, to_node, cb);

    // And recurse
    if (!from_node->leaf()) {
        for(Index idx = 0; idx < from_node->size(); idx++) {
            RTree_lift_cut_nodes_from_tree(from_node->node(idx), to_node, cb);
        }
    }
}


template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_verify_no_cut_nodes(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
#ifdef PROXDEBUG
    assert(node->parent() == NULL || node->cutNodesSize() == 0);
#endif
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_verify_no_cut_nodes_in_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
#ifdef PROXDEBUG
    RTree_verify_no_cut_nodes(node);
    if (!node->leaf())
        for(typename RTreeNodeType::Index idx = 0; idx < node->size(); idx++)
            RTree_verify_no_cut_nodes_in_tree(node->node(idx));
#endif
}

#ifdef LIBPROX_LIFT_CUTS
/* Takes a leaf node from which an object has been removed and, if it contains too few nodes,
 *  redistributes the objects it contains and removes the node from the tree.
 *  Returns the new root (which it may create because it might have to reinsert objects, which
 *  can itself cause a new root to appear.
 *
 *  The basic approach is to follow collapses up the tree until we
 *  find the highest node we want to collapse. We then move cuts out
 *  of the way, collect all objects within that part of the tree, and
 *  reinsert all the objects. This approach can only work with
 *  cut-lifting since it has to move everything out of the way and
 *  rebuild a whole part of the tree.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_condense_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf,
    LocationServiceCache<SimulationTraits>* loc,
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
        if (child_count < 1) { // FIXME should be some larger value,
                               // but note warning below about how
                               // changing this could break the code
                               // below
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
        RTree_lift_cut_nodes_from_tree(highest_removed, parent, cb);
        RTree_verify_no_cut_nodes_in_tree(highest_removed);

        // Then, remove nodes
        std::queue<RTreeNodeType*> removedNodes;
        parent->erase(highest_removed, cb);
        removedNodes.push(highest_removed);

        // FIXME this could reinsert entire nodes instead of individual objects, but we'd need a better idea of how to actually accomplish that...
        while(!removedNodes.empty()) {
            RTreeNode<SimulationTraits, NodeData, CutNode>* removed = removedNodes.front();
            removedNodes.pop();

            // BEWARE that this ordering only currently works because
            // leaves are guaranteed to be empty because of the
            // condition in the initial loop finding what to
            // remove. The aggregateChildRemoved/aggregateChildAdded
            // order is broken here because the object should be
            // cleared out of the old node before being
            // reinserted. This works for nodes because they are
            // queued up for processing later, so the addition for
            // them doesn't happen immediately here.
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
            removed->destroy(loc, cb);
        }
    }


    // After removal, there's a chance that the root node ended up with no
    // elements, in which case it should be marked as a leaf node
    if (root->size() == 0)
        root->leaf(true);

    // Perform recomputation of node data
    n = recompute_start;
    while(n != NULL) {
        n->recomputeData(loc, t, cb);
        n = n->parent();
    }

    return root;
}

#else // not LIBPROX_LIFT_CUTS

/*  Takes a leaf node from which an object has been removed and, if it contains too few nodes,
 *  redistributes the objects it contains and removes the node from the tree.
 *  Returns the new root (which it may create because it might have to reinsert objects, which
 *  can itself cause a new root to appear.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_condense_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename RTreeNodeType::Index Index;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    // We'll work our way up the tree looking for parent nodes which
    // have a total # of grandchildren that fit within the node
    // itself, i.e. nodes where it doesn't make sense to keep the
    // children.

    // The current node and its parent
    RTreeNodeType* n = leaf;
    RTreeNodeType* parent = n->parent();
    // Whether we've changed anything, requiring recomputation of
    // bounds
    bool dirty_bounds = false;
    while(parent != NULL) {
        // Decide whether we have few enough grandchildren to
        // merge. They also need to all be of the same type (nodes or
        // objects).
        int gchildren = 0;
        bool all_nodes = true, all_objects = true;
        for(Index ci = 0; ci < parent->size(); ci++) {
            gchildren += parent->node(ci)->size();
            all_nodes = all_nodes && !parent->node(ci)->leaf();
            all_objects = all_objects && parent->node(ci)->leaf();
        }
        // We use half capacity to avoid being too aggressive about
        // splitting/merging. We can only merge if all grandchildren
        // match types
        if (gchildren <= parent->capacity()/2 && (all_nodes || all_objects)) {
            // We're getting rid of all the children. We need to
            // handle a number of things properly: updating cuts,
            // making sure aggregates are updated/destroyed properly,
            // and just the book keeping for moving the objects into
            // the parent node.

            // We're removing the middle level of nodes. Cuts that are in the
            // parent or the children should be fine, but any that cut through
            // these middle nodes will need to be moved. The order of
            // grandchildren is preserved so that cuts that go through only
            // grandchildren (or lower nodes) do not have to be adjusted.
            //
            // We're technically going to lift the cuts, but this is a very
            // limited form of lifting: instead of lifting all cuts within the
            // subtree, we only lift ones that are in affected nodes. This means
            // we're mostly just shifting cuts up by one level (although it is
            // possible for this change to be high in the tree, affect a cut
            // that goes through one middle-level node, and then goes much
            // deeper in some other branch in this same subtree).
            for(Index ci = 0; ci < parent->size(); ci++) {
                RTree_lift_cut_nodes(parent->node(ci), parent, cb);
            }

            // Remove the children nodes, keeping track of them
            std::vector<RTreeNodeType*> child_nodes;
            while(!parent->empty()) {
                child_nodes.push_back(parent->node( parent->size()-1 ));
                parent->erase(child_nodes.back(), cb);
            }

            // Make sure the parent is the right type of node. It
            // should be empty now. We also explicitly ask for clear()
            // because this resets the bounds data, which will then be
            // updated as we insert the objects
            assert(parent->empty());
            parent->clear(loc, cb);
            parent->leaf(all_objects);

            // Then, for each child, remove it's children and add them
            // to the parent. Destroy the node as it's no longer needed. These
            // need to remain in the same order as the grandchildren in order to
            // avoid moving cuts that don't move through the level of RTreeNodes
            // that are being removed.
            for(Index ci = 0; ci < child_nodes.size(); ci++) {
                // Note that we need correct remove -> add ordering so
                // aggregate listeners don't get confused. We need to
                // copy each entry out, remove it, then add it to the
                // parent.
                while(!child_nodes[ci]->empty()) {
                    if (child_nodes[ci]->leaf()) {
                        LocCacheIterator gchild_object = child_nodes[ci]->object(0).object;
                        child_nodes[ci]->erase(loc, gchild_object, true, cb);
                        parent->insert(loc, gchild_object, t, cb);
                    }
                    else {
                        RTreeNodeType* gchild_node = child_nodes[ci]->node(0);
                        child_nodes[ci]->erase(gchild_node, cb);
                        parent->insert(gchild_node, cb);
                    }
                }
                child_nodes[ci]->destroy(loc, cb);
            }

            // Force recomputation of bounds as we move up the rest of
            // the tree.
            dirty_bounds = true;
        }
        else if (n->empty()) {
            // Even if we couldn't merge all siblings into the grandparent, this
            // node might still be empty.

            // Get cuts out of this node. TODO(we could probably improve
            // efficiency a bit by not lifting this up a node but just telling
            // the cut to destroy the cutnode in this node
            RTree_lift_cut_nodes(n, parent, cb);

            // Remove from parent and destroy
            parent->erase(n, cb);
            n->destroy(loc, cb);

            // Force bounds recomputation on parents
            dirty_bounds = true;
        }

        // Move on to next node
        n = parent;
        parent = n->parent();

        // Update bounds if necessary. Do this after moving to the
        // parent so that when we do merge nodes we don't recompute
        // the just-computed bounds.
        if (dirty_bounds && parent != NULL)
            parent->recomputeData(loc, t, cb);
    }

    // We need to return the new root. The loop above should have
    // taken us there, so the current node should be the root.
    return n;
}

#endif

/* Update nodes up the tree to fix node information due to a change. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_update_up_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    LocationServiceCache<SimulationTraits>* loc,
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
    LocationServiceCache<SimulationTraits>* loc,
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
    LocationServiceCache<SimulationTraits>* loc,
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

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_collect_cuts(RTreeNode<SimulationTraits, NodeData, CutNode>* node, std::tr1::unordered_set<typename CutNode::CutType*>* cuts)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename CutNode::CutType Cut;

    for(typename RTreeNodeType::CutNodeListIterator it = node->cutNodesBegin(); it != node->cutNodesEnd(); it++) {
        Cut* cut = it->first;
        cuts->insert(cut);
    }

    if (node->leaf())
        return;

    for(int i = 0; i < node->size(); i++) {
        RTreeNodeType* child_node = node->node(i);
        RTree_collect_cuts(child_node, cuts);
    }
}

/* Recursively report the bounds tightness of nodes. This is basically a
 * read-only version of tree restructuring. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_report_bounds(
    FILE* fout,
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t)
{
    fprintf(fout, "{ ");

    // Report this nodes volume
    float this_volume = root->data().volume();
    fprintf(fout, " \"volume\" : %f, \"children\" : [ ", this_volume);
    // Recurse
    if (root->leaf()) {
        for(int i = 0; i < root->size(); i++) {
            if (i > 0) fprintf(fout, ", ");
            fprintf(fout, "{ \"volume\" : %f }", root->childData(i, loc, t).volume());
        }
    }
    else {
        for(int i = 0; i < root->size(); i++) {
            if (i > 0) fprintf(fout, ", ");
            RTree_report_bounds(fout, root->node(i), loc, t);
        }
    }

    fprintf(fout, " ] }");
}

/* Cleanup performed after deletion of an object/node from a node. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_post_deletion_cleanup(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    RTreeNode<SimulationTraits, NodeData, CutNode>* clean_start,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    RTreeNodeType* new_root = RTree_condense_tree(clean_start, loc, t, cb);

    // We might need to shorten the tree if the root is left with only one child.
    if (!root->leaf() && root->size() == 1) {
        new_root = root->node(0);
        // Notify cuts so they can refine to the new root
        if (cb.rootReplaced) {
            for(typename RTreeNodeType::CutNodeListConstIterator cut_it = root->cutNodesBegin(); cut_it != root->cutNodesEnd(); ) {
                CutNode* cutnode = cut_it->second;
                cut_it++; // Advance now to avoid invalidating iterator in callback
                cb.rootReplaced(cutnode, root, new_root);
            }
        }
        new_root->parent(NULL);
        root->destroy(loc, cb);
    }
    return new_root;
}

/* Deletes the object from the given tree.  Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_delete_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t,
    bool temporary,
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
    leaf_with_obj->erase(loc, obj_id, temporary, cb);

    return RTree_post_deletion_cleanup(root, leaf_with_obj, loc, t, cb);
}


/* Deletes the object from the given tree.  Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_delete_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    LocationServiceCache<SimulationTraits>* loc,
    const typename SimulationTraits::TimeType& t,
    bool temporary,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    RTreeNodeType* parent = node->parent();
    if (parent == NULL)
        return root;

    // Notify any cuts that the node is leaving
    assert(!parent->leaf());
    parent->erase(node, cb);

    return RTree_post_deletion_cleanup(root, node, loc, t, cb);
}


/* Collect a list of objects (in the form of location cache iterators) from
 * this tree. Useful when you need to restructure the entire tree.
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_collect_objects(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    std::vector<typename LocationServiceCache<SimulationTraits>::Iterator>* objects
)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;

    if (root->leaf()) {
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
            objects->push_back(root->object(i).object);
    }
    else {
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
            RTree_collect_objects(root->node(i), objects);
    }
}

/* Destroys an entire subtree. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_destroy_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    LocationServiceCache<SimulationTraits>* loc,
    const typename RTreeNode<SimulationTraits, NodeData, CutNode>::Callbacks& cb)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;

    if (!root->leaf()) {
        while(root->size()) {
            RTreeNodeType* child = root->erasePop(cb);
            RTree_destroy_tree(child, loc, cb);
        }
    }

    root->destroy(loc, cb);
}

} // namespace Prox

#endif //_PROX_RTREE_CORE_HPP_
