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

// Currently the following two are arbitrarily selected parameters.
// Need to explore this parameter space.

#define kShapeParameter 0.5f
#define kGeometryParameter (1.0 - kShapeParameter)

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
struct RTreeObjectNode : public CutNodeContainer<CutNode> {
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    LocCacheIterator object;

    RTreeObjectNode(LocCacheIterator it)
     : object(it)
    {}

};

template<typename SimulationTraits, typename NodeData, typename CutNode>
class RTree;

template<typename SimulationTraits, typename NodeData, typename CutNode>
struct RTreeNode : public CutNodeContainer<CutNode> {
public:
    typedef RTree<SimulationTraits, NodeData, CutNode> RTreeType;

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

    typedef RTreeObjectNode<SimulationTraits, CutNode> ObjectNode;

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
    // Object was removed from a leaf node. emptiedNode indicates whether the
    // removal caused the node to become empty. This can be relevant to tracking
    // cuts since the cut may be need to be moved up (node ID inserted in
    // results) along with the removal of the child object.
    typedef std::tr1::function<void(CutNode*, const LocCacheIterator&, bool permanent, bool emptiedNode)> ObjectRemovedCallback;

    // Node was added somewhere above the cut, e.g. because we are
    // querying a replicated tree and we saw a node addition. The cut
    // node "closest" to the left of the new node will be the one this
    // callback is invoked for. This will always be a sibling of the
    // new node since this callback is never necessary when inserting
    // into a node with no children (you couldn't have a cut below
    // this node in that case). The node will have been added already,
    // so you can use the structure of the updated tree (e.g. getting
    // the parent) to complete the update.
    typedef std::tr1::function<void(CutNode*, RTreeNode*)> NodeAddedAboveCutCallback;
    // Node is being removed (completely destroyed), but the cut is still
    // passing through it (because we're not lifting cuts). The node is just
    // being removed and should be empty, so handling this should be as simple
    // as removing the cut node and reporting it to any listeners. This will
    // *not* be invoked if the the node that needs to be removed is the last
    // child of its parent: in that case, cuts will have to do the same thing as
    // if we called the LiftCutCallback, so we just invoke that to keep
    // NodeRemovedCallback implementations simpler.
    typedef std::tr1::function<void(CutNode*, RTreeNode*)> NodeRemovedCallback;
    // Callback to invoke when a node (and it's subtree) needs to be
    // reparented. Same signature is used for reparenting that happens
    // along the cut (for geometric cut-based queries) and above the
    // cut (for replication)
    typedef std::tr1::function<void(CutNode*, RTreeNode* /*child*/, RTreeNode* /*old_parent*/, RTreeNode* /*new_parent*/)> NodeReparentedCallback;
    // Same as above, but for objects. For objects reparenting can't
    // happen above the cut (that's nonsensical), it's just an
    // alternative to the remove/add sequence.
    typedef std::tr1::function<void(CutNode*, const LocCacheIterator& /*child*/, RTreeNode* /*old_parent*/, RTreeNode* /*new_parent*/)> ObjectReparentedCallback;


    typedef uint16 Index;

    struct Callbacks {
        AggregatorType* aggregator;
        AggregateListenerType* aggregate;
        ObjectLeafChangedCallback objectLeafChanged;
        GetObjectLeafCallback getObjectLeaf;
        RootReplacedByChildCallback rootReplaced;
        RootReplacedByChildCallback replicatedRootCreated;
        RootReplacedByChildCallback replicatedRootDestroyed;
        NodeSplitCallback nodeSplit;
        NodeSplitCallback nodeSplitFinished;
        // For tree replication, called back for cuts which have been refined
        // past the node that split, but which are tracking it still and so need
        // to know about the new node.
        NodeSplitCallback replicatedNodeSplit;
        LiftCutCallback liftCut;
        ReorderCutCallback reorderCut;
        ObjectInsertedCallback objectInserted;
        ObjectRemovedCallback objectRemoved;
        // When not lifting cuts, we need a few additional types of
        // callbacks
        NodeAddedAboveCutCallback nodeAddedAboveCut;
        NodeRemovedCallback nodeWithCutRemoved;
        // Reparenting callbacks, important when not using cut
        // lifting, and especially important when using replication
        NodeReparentedCallback nodeReparented;
        NodeReparentedCallback nodeReparentedAboveCut;
        ObjectReparentedCallback objectReparented;

    };
private:
    static const uint8 ObjectsFlag = 0x02; // elements are object pointers instead of node pointers

    RTreeType* mOwner;
    union {
        RTreeNode** nodes;
        ObjectNode* objects;
        uint8* magic;
    } elements;
    RTreeNode* mParent;
    NodeData mData;
    uint8 flags;
    Index count;
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

        void insert(RTreeNode* parent, RTreeNode* newchild, const Time&) {
            parent->insert(newchild);
        }

        void reparent(RTreeNode* old_parent, RTreeNode* new_parent, RTreeNode* child, const Time&) {
            // Currently we only support reparenting between siblings,
            // e.g. because of splits. This is an important
            // assumption when notifying cuts and guaranteeing we
            // maintain proper cuts.
            assert(old_parent->parent() == new_parent->parent());
            new_parent->reparent(child, old_parent);
        }
    };

    struct ObjectChildOperations {
        const ObjectNode& child(RTreeNode* parent, int idx) {
            return parent->object(idx);
        }

        const LocCacheIterator& childData(RTreeNode* parent, int idx) {
            return parent->object(idx).object;
        }

        NodeData data(LocationServiceCacheType* loc, const LocCacheIterator& child, const Time& t) {
            return NodeData(loc, child, t);
        }

        void insert(RTreeNode* parent, const LocCacheIterator& newchild, const Time& t) {
            parent->insert(newchild, t);
        }

        void reparent(RTreeNode* old_parent, RTreeNode* new_parent, const LocCacheIterator& child, const Time& t) {
            // Currently we only support reparenting between siblings,
            // e.g. because of splits. This is an important
            // assumption when notifying cuts and guaranteeing we
            // maintain proper cuts.
            assert(old_parent->parent() == new_parent->parent());
            new_parent->reparent(child, t, old_parent);
        }
    };


    RTreeNode(RTreeType* _owner)
     : mOwner(_owner), mParent(NULL),
       mData(), flags(0), count(0),
       aggregate( ObjectIDRandom()() )
    {
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(ObjectNode) );
        uint32 magic_size = max_element_size * capacity();
        elements.magic = new uint8[magic_size];
        memset(elements.magic, 0, magic_size);

        objectChildren(true);

        if (callbacks().aggregate != NULL) {
            // Make sure we get it into the location service cache first, even
            // if it's just a placeholder.
            _owner->loc()->addPlaceholderImposter(
                aggregate,
                data().getBoundsCenter(), data().getBoundsCenterBoundsRadius(), data().getBoundsMaxObjectSize(),
                // Zernike descriptor and mesh are just default value placeholders
                "", ""
            );
            callbacks().aggregate->aggregateCreated(callbacks().aggregator, aggregate);
        }
    }

    // This version initializes the node from known data, i.e. for
    // replicating existing trees.
    RTreeNode(RTreeType* _owner, const LocCacheIterator& node, const Time& t)
     : mOwner(_owner), mParent(NULL),
       mData(_owner->loc(), node, t), flags(0), count(0),
       aggregate( _owner->loc()->iteratorID(node) )
    {
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(ObjectNode) );
        uint32 magic_size = max_element_size * capacity();
        elements.magic = new uint8[magic_size];
        memset(elements.magic, 0, magic_size);

        objectChildren(true);

        // FIXME(ewencp) Better initialization of mData? Included max object
        // size but overwrite it for aggregates?

        if (callbacks().aggregate != NULL) {
            // Make sure we get it into the location service cache first, even
            // if it's just a placeholder.
            _owner->loc()->addPlaceholderImposter(
                aggregate,
                data().getBoundsCenter(), data().getBoundsCenterBoundsRadius(), data().getBoundsMaxObjectSize(),
                // Zernike descriptor and mesh are just default value placeholders
                "", ""
            );
            callbacks().aggregate->aggregateCreated(callbacks().aggregator, aggregate);
        }
    }

    // We have a destroy method and hide the destructor in private in order to
    // ensure the aggregate callbacks get invoked properly.
    void destroy() {
        // Make sure we get all the destructors right by just clearing the
        // entire node
        clear();

        if (callbacks().aggregate != NULL) callbacks().aggregate->aggregateDestroyed(callbacks().aggregator, aggregate);
        delete this;
    }

private:
    ~RTreeNode() {
        delete[] elements.magic;
        assert(CutNodeContainer<CutNode>::cuts.size() == 0);
    }

public:
    RTreeType* owner() const { return mOwner; }
    Index capacity() const { return owner()->elementsPerNode(); }
    bool replicated() const { return owner()->replicated(); }
    LocationServiceCacheType* loc() const { return owner()->loc(); }
    const Callbacks& callbacks() const { return owner()->callbacks(); }

    // Whether we hold objects or other RTreeNodes as children. Note that this
    // is different from leaf because replicated trees may have no objectChildren()
    // nodes but they will have leaves
    bool objectChildren() const {
        return (flags & ObjectsFlag);
    }
    void objectChildren(bool d) {
        flags = (flags & ~ObjectsFlag) | (d ? ObjectsFlag : 0x00);
    }

    bool empty() const {
        return (count == 0);
    }
    bool full() const {
        return (count >= capacity());
    }
    Index size() const {
        return count;
    }

    // Size of this tree, including this node.
    int treeSize() const {
        int result = 1;
        // For leaves, we count the children ourselves
        if (objectChildren())
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

    RTreeNode* root() const {
        RTreeNode* r = const_cast<RTreeNode*>(this);
        while(r->parent() != NULL)
            r = r->parent();
        return r;
    }

    const ObjectNode& object(int i) const {
        assert( objectChildren() );
        assert( i < count );
        return elements.objects[i];
    }

    RTreeNode* node(int i) const {
        assert( !objectChildren() );
        assert( i < count );
        return elements.nodes[i];
    }

    const NodeData& data() const {
        return mData;
    }

    NodeData childData(int i, const Time& t) {
        if (objectChildren())
            return NodeData(loc(), object(i).object, t);
        else
            return node(i)->data();
    }

    void updateReplicatedNodeData(const LocCacheIterator& node_locit, const Time& t) {
        assert(owner()->replicated());
        assert(loc()->iteratorID(node_locit) == aggregate);

        BoundingSphere orig = mData.getBounds();
        mData = NodeData(loc(), node_locit, t);
        BoundingSphere updated = mData.getBounds();

        if (callbacks().aggregate != NULL && updated != orig) {
            callbacks().aggregate->aggregateBoundsUpdated(
                callbacks().aggregator, aggregate,
                mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
            );
        }
    }

    void recomputeData(const Time& t) {
        // We need to be careful about recomputing data to handle
        // replicas properly. For now, our policy is that for replicas
        // we can recompute data automatically (rather than getting
        // updates from the server) as long as we know it is
        // safe. This means that we must have the children of the node
        // -- we have to trust updates for replicated nodes on the cut
        // (otherwise, with no children, we would just clear out their
        // data), but for other things we can assume our current set
        // of children reflects the real tree and it is therefore safe
        // to recompute.
        if (replicated() && empty()) return;

        BoundingSphere orig = mData.getBounds();
        mData = NodeData();
        for(int i = 0; i < size(); i++)
          mData.mergeIn( childData(i, t), size() );
        BoundingSphere updated = mData.getBounds();
        if (callbacks().aggregate != NULL && updated != orig) {
            callbacks().aggregate->aggregateBoundsUpdated(
                callbacks().aggregator, aggregate,
                mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
            );
        }
    }

private:
    void notifyRemoved(const LocCacheIterator& obj, bool permanent) {
        if (callbacks().objectRemoved)
            RTree_notify_cuts(this, callbacks().objectRemoved, obj, permanent, empty());
    }

public:

    void clear() {
        Index old_count = count;

        // If we have child objects, we need to notify cuts of removal
        // To allow validation after notifyRemoved we need to work 1 at a time.
        if (objectChildren()) {
            for(int i = 0; i < old_count; i++) {
                count = old_count-i-1;
                if (callbacks().aggregate != NULL) {
                    ObjectID obj_id = loc()->iteratorID(this->elements.objects[old_count-i-1].object);
                    callbacks().aggregate->aggregateChildRemoved(
                        callbacks().aggregator, aggregate, obj_id,
                        mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
                    );
                }
                notifyRemoved(this->elements.objects[old_count-i-1].object, false);
                // Explicitly call destructor, necessary since we use placement
                // new to handle constructing objects in place (see insert())
                this->elements.objects[old_count-i-1].~ObjectNode();
            }
        }
        else {
            // If we're removing child nodes, we don't need to notify
            // of object removal, but we do need to notify of
            // aggregate child removal
            for(Index i = 0; i < count; i++) {
                if (callbacks().aggregate != NULL) {
                    callbacks().aggregate->aggregateChildRemoved(
                        callbacks().aggregator, aggregate, node(i)->aggregate,
                        mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
                    );
                }
            }
            count = 0;
        }

#ifdef PROXDEBUG
        // Clear out data for safety
        uint32 max_element_size = std::max( sizeof(RTreeNode*), sizeof(ObjectNode) );
        for(uint32 i = 0; i < max_element_size * capacity(); i++)
            elements.magic[i] = 0;
#endif

        mData = NodeData();
    }

private:
    Index insertWithoutNotification(const LocCacheIterator& obj, const Time& t) {
        assert (count < capacity());
        assert (objectChildren() == true);

        int idx = count;
        // Use placement new to get the constructor called without using
        // assignment (which would result in a destructor on existing (likely
        // bogus or leftover) data being called. Careful destruction handled in
        // erase() and clear.
        new (&(elements.objects[idx])) ObjectNode(obj);
        callbacks().objectLeafChanged(obj, this);
        count++;
        mData.mergeIn( NodeData(loc(), obj, t), size() );

        if (callbacks().aggregate != NULL) {
            callbacks().aggregate->aggregateChildAdded(
                callbacks().aggregator, aggregate, loc()->iteratorID(obj),
                mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
            );
        }
        return idx;
    }
public:
    void insert(const LocCacheIterator& obj, const Time& t) {
        Index idx = insertWithoutNotification(obj, t);
        if (callbacks().objectInserted)
            RTree_notify_cuts(this, callbacks().objectInserted, obj, idx);
    }

    /** Insert node as a child and invoke callbacks if necessary. If
     *  provided, after indicates where in the list the node should
     *  appear. The order of existing items are preserved (everything
     *  is shifted rather than using swaps); this operation is safe
     *  for cuts as long as you take care of handling the inserted
     *  node. If not specified, the node is added at the end of the
     *  current list.
     */
    void insert(RTreeNode* node, RTreeNode* after = NULL) {
        assert (count < capacity());
        assert (objectChildren() == false);
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
        mData.mergeIn( node->data(), size() );

        if (callbacks().aggregate != NULL) {
            callbacks().aggregate->aggregateChildAdded(
                callbacks().aggregator, aggregate, node->aggregate,
                mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
            );
        }
    }

    // NOTE: does not recalculate the bounding sphere
    void erase(const LocCacheIterator& obj, bool temporary) {
        eraseWithoutNotification(obj);

        // This is only invoked due to the deletion of an individual object, so
        // we specify permanent deletion (last param)
        notifyRemoved(obj, !temporary);
    }

private:
    void eraseWithoutNotification(const LocCacheIterator& obj) {
        assert(count > 0);
        assert(objectChildren() == true);

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
        this->elements.objects[count-1].~ObjectNode();
        // Finally, reduce the count
        count--;

        if (callbacks().aggregate != NULL) {
            ObjectID obj_id = loc()->iteratorID(obj);
            callbacks().aggregate->aggregateChildRemoved(
                callbacks().aggregator, aggregate, obj_id,
                mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
            );
        }
    }

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
    void erase(const RTreeNode* node) {
        assert(count > 0);
        assert(objectChildren() == false);
        // find node
        Index node_idx;
        for(node_idx = 0; node_idx < count; node_idx++)
            if (elements.nodes[node_idx] == node) break;
        erase(node_idx);

        if (callbacks().aggregate != NULL) {
            callbacks().aggregate->aggregateChildRemoved(
                callbacks().aggregator, aggregate, node->aggregate,
                mData.getBoundsCenter(), mData.getBoundsCenterBoundsRadius(), mData.getBoundsMaxObjectSize()
            );
        }
    }

    // Removes the last child in the node and returns the pointer to it.
    // NOTE: does not recalculate the bounding sphere.
    RTreeNode* erasePop() {
        assert(count > 0);
        assert(objectChildren() == false);
        int idx = size() - 1;
        RTreeNode* retval = node(idx);
        erase(idx);
        return retval;
    }

    // Reparenting allows you to move nodes/objects between parent
    // nodes. For objects its the same as an add/remove, but for nodes
    // it allows moving entire subtrees around. Functionally from the
    // perspective of the RTree it's still just an add/remove, but the
    // notifications to cuts work differently
    void reparent(RTreeNode* child, RTreeNode* old_parent) {
        old_parent->erase(child);
        insert(child);
        // Cuts at and below the reparented node may need to know that
        // it was updated
        if (callbacks().nodeReparented)
            RTree_notify_cuts(child, callbacks().nodeReparented, child, old_parent, this);
        if (callbacks().nodeReparentedAboveCut)
            RTree_notify_descendant_cuts(child, callbacks().nodeReparentedAboveCut, child, old_parent, this);
    }

    void reparent(const LocCacheIterator& obj, const Time& t, RTreeNode* old_parent) {
        // Get rid of the old one, but don't notify.
        old_parent->eraseWithoutNotification(obj);
        // Add to this node
        insertWithoutNotification(obj, t);

        // Cuts passing through the old parent may need to know that
        // the object moved.
        if (callbacks().objectReparented)
            RTree_notify_cuts(old_parent, callbacks().objectReparented, obj, old_parent, this);
    }

    bool contains(const LocCacheIterator& obj) const {
        assert(objectChildren());
        for(Index obj_idx = 0; obj_idx < count; obj_idx++)
            if (elements.objects[obj_idx] == obj) return true;
        return false;
    }
    bool contains(RTreeNode* node) const {
        assert(!objectChildren());
        for(Index idx = 0; idx < count; idx++)
            if (elements.nodes[idx] == node) return true;
        return false;
    }


    Index indexOf(const LocCacheIterator& obj) const {
        assert(contains(obj));
        assert(objectChildren());
        for(Index idx = 0; idx < count; idx++)
            if (elements.objects[idx] == obj) return idx;
        return -1;
    }
    Index indexOf(RTreeNode* node) const {
        assert(contains(node));
        assert(!objectChildren());
        for(Index idx = 0; idx < count; idx++)
            if (elements.nodes[idx] == node) return idx;
        return -1;
    }

    RTreeNode* selectBestChildNode(const LocCacheIterator& obj, const Time& t) {
        return NodeData::selectBestChildNode(this, loc(), obj, t);
    }

    static RTreeNode* selectBestChildNodeFromPair(RTreeNode* n1, RTreeNode* n2, const LocCacheIterator& obj, const Time& t) {
        return NodeData::selectBestChildNodeFromPair(n1, n2, n1->loc(), obj, t);
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
    typedef typename SimulationTraits::realType Real;
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


    BoundingSphereDataBase(LocationServiceCacheType* loc, const LocCacheIterator& obj,  const Time& t)
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
    void mergeIn(const NodeData& other, uint32 currentChildrenCount) {
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
    static RTreeNodeType* selectBestChildNodeFromPair(RTreeNodeType* n1, RTreeNodeType* n2, LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
        float min_increase = 0.f;
        RTreeNodeType* min_increase_node = NULL;

        BoundingSphere obj_bounds = loc->worldCompleteBounds(obj_id, t);

        RTreeNodeType* nodes[2] = { n1, n2 };
        for(int i = 0; i < 2; i++) {
          RTreeNodeType* child_node = nodes[i];
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
    static void pickNextChild(std::vector<NodeData>& split_data, const SplitGroups& split_groups, const NodeData& group_data_0, const NodeData& group_data_1, int32* next_child, int32* selected_group)  {

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

    /** Gets the center of the aggregate bounds object. */
    Vector3 getBoundsCenter() const {
        return bounding_sphere.center();
    }
    /** Gets the center positoin bounds radius (radius of bounds around center
     *  points, not around all subobjects).
     */
    Real getBoundsCenterBoundsRadius() const {
        // Without tracking both things, we can't actually differentiate the
        // center bounds radius and the max object size. To make this actually
        // looks sane for objects, we just have to ignore center bounds radius
        // for aggregates and pretend they are giant objects.
        return 0;
    }
    /** Gets the maximum object size under this node. */
    Real getBoundsMaxObjectSize() const {
        return bounding_sphere.radius();
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
    typedef typename SimulationTraits::realType Real;
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
      : BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>(loc, obj_id, t),
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
  void mergeIn(const NodeData& other, uint32 currentChildrenCount) {
    BoundingSphereDataBase<SimulationTraits, MaxSphereData, CutNode>::mergeIn(other, currentChildrenCount);
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
    static RTreeNodeType* selectBestChildNodeFromPair(RTreeNodeType* n1, RTreeNodeType* n2, LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
        float min_increase = 0.f;
        RTreeNodeType* min_increase_node = NULL;

        BoundingSphere obj_bounds = loc->worldRegion(obj_id, t);
        float obj_max_size = loc->maxSize(obj_id);

        RTreeNodeType* nodes[2] = { n1, n2 };
        for(int i = 0; i < 2; i++) {
            RTreeNodeType* child_node = nodes[i];
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

    Vector3 getBoundsCenter() const {
        return ThisBase::bounding_sphere.center();
    }
    Real getBoundsCenterBoundsRadius() const {
        return ThisBase::bounding_sphere.radius();
    }
    Real getBoundsMaxObjectSize() const {
        return mMaxRadius;
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


/* Maintains the largest bounding sphere radius as well as the hierarchical bounding sphere.
   But it also tries to group similar objects together as much as it can based on their zernike
   descriptors.
*/
template<typename SimulationTraits, typename CutNode>
class SimilarMaxSphereData : public BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData<SimulationTraits, CutNode>, CutNode> {
  public:
    typedef SimilarMaxSphereData NodeData; // For convenience/consistency
    typedef BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData<SimulationTraits, CutNode>, CutNode> ThisBase;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;
    typedef typename SimulationTraits::TimeType Time;
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    SimilarMaxSphereData()
      : BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData, CutNode>(),
        mMaxRadius(0.f), zernike_descriptor(ZernikeDescriptor::null()),
        mesh(""), descriptorReader(DescriptorReader::getDescriptorReader())
    {

    }

    SimilarMaxSphereData(LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t)
      : BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData, CutNode>(loc, obj_id, t),
        mMaxRadius( loc->maxSize(obj_id) ), zernike_descriptor(loc->zernikeDescriptor(obj_id)),
        mesh(loc->mesh(obj_id)), descriptorReader(DescriptorReader::getDescriptorReader())
    {
      // Note: we override this here because we need worldCompleteBounds for
      // just the bounds data, but with the max size values, we can use the
      // smaller worldRegion along with the maximum size object.  Note
      // difference in satisfiesConstraints
      ThisBase::bounding_sphere = loc->worldRegion(obj_id, t);
      zernike_descriptor = descriptorReader->getZernikeDescriptor(mesh);
    }

    NodeData merge(const NodeData& other) const {
      NodeData result = BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData, CutNode>::merge(other);
      result.mMaxRadius = std::max( mMaxRadius, other.mMaxRadius );
      return result;
    }

    // Merge the given info into this info
    void mergeIn(const NodeData& other, uint32 currentChildrenCount) {
      BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData, CutNode>::mergeIn(other, currentChildrenCount);
      mMaxRadius = std::max( mMaxRadius, other.mMaxRadius );

      zernike_descriptor = zernike_descriptor.multiply(currentChildrenCount-1).plus(other.zernike_descriptor).divide(currentChildrenCount);
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
      //the metric used to choose the best child node.
      float min_metric = FLT_MAX;

      RTreeNodeType* chosen_node = NULL;

      float obj_max_size = loc->maxSize(obj_id);
      BoundingSphere obj_bounds = loc->worldCompleteBounds(obj_id, t);
      const ZernikeDescriptor& new_zd = loc->zernikeDescriptor(obj_id);

      float normalizer = 0.00000;
      for (int i=0; i<node->size(); i++) {
	RTreeNodeType* child_node = node->node(i);
        BoundingSphere merged = child_node->data().bounding_sphere.merge(obj_bounds);
	float new_max_size = std::max(child_node->data().mMaxRadius, obj_max_size);
        BoundingSphere old_total( child_node->data().bounding_sphere.center(), child_node->data().bounding_sphere.radius() + child_node->data().mMaxRadius );
        BoundingSphere total(merged.center(), merged.radius() + new_max_size);

	if (total.volume() > normalizer) normalizer = total.volume();
      }

      //trying to balance between choosing far-away objects for grouping and choosing
      //similar objects for grouping.
      for (int i=0; i<node->size(); i++) {
        RTreeNodeType* child_node = node->node(i);
        BoundingSphere merged = child_node->data().bounding_sphere.merge(obj_bounds);
        float new_max_size = std::max(child_node->data().mMaxRadius, obj_max_size);
        BoundingSphere old_total( child_node->data().bounding_sphere.center(), child_node->data().bounding_sphere.radius() + child_node->data().mMaxRadius );
        BoundingSphere total(merged.center(), merged.radius() + new_max_size);
        float ns_minus_os =  (total.volume() - old_total.volume())/normalizer ;

        ZernikeDescriptor median_zd = child_node->data().zernike_descriptor;

        float nz_minus_mz = median_zd.minus(new_zd).l2Norm();

        float metric = kGeometryParameter * ns_minus_os + kShapeParameter * nz_minus_mz;

        if (chosen_node == NULL || metric < min_metric) {
          min_metric = metric;
          chosen_node = child_node;
        }
      }

      return chosen_node;
    }
    static RTreeNodeType* selectBestChildNodeFromPair(RTreeNodeType* n1, RTreeNodeType* n2, LocationServiceCacheType* loc, const LocCacheIterator& obj_id, const Time& t) {
      //the metric used to choose the best child node.
      float min_metric = FLT_MAX;

      RTreeNodeType* chosen_node = NULL;

      float obj_max_size = loc->maxSize(obj_id);
      BoundingSphere obj_bounds = loc->worldCompleteBounds(obj_id, t);
      const ZernikeDescriptor& new_zd = loc->zernikeDescriptor(obj_id);

      float normalizer = 0.00000;
      RTreeNodeType* nodes[2] = { n1, n2 };
      for (int i=0; i<2; i++) {
        RTreeNodeType* child_node = nodes[i];
        BoundingSphere merged = child_node->data().bounding_sphere.merge(obj_bounds);
	float new_max_size = std::max(child_node->data().mMaxRadius, obj_max_size);
        BoundingSphere old_total( child_node->data().bounding_sphere.center(), child_node->data().bounding_sphere.radius() + child_node->data().mMaxRadius );
        BoundingSphere total(merged.center(), merged.radius() + new_max_size);

	if (total.volume() > normalizer) normalizer = total.volume();
      }

      //trying to balance between choosing far-away objects for grouping and choosing
      //similar objects for grouping.
      for (int i=0; i<2; i++) {
        RTreeNodeType* child_node = nodes[i];
        BoundingSphere merged = child_node->data().bounding_sphere.merge(obj_bounds);
        float new_max_size = std::max(child_node->data().mMaxRadius, obj_max_size);
        BoundingSphere old_total( child_node->data().bounding_sphere.center(), child_node->data().bounding_sphere.radius() + child_node->data().mMaxRadius );
        BoundingSphere total(merged.center(), merged.radius() + new_max_size);
        float ns_minus_os =  (total.volume() - old_total.volume())/normalizer ;

        ZernikeDescriptor median_zd = child_node->data().zernike_descriptor;

        float nz_minus_mz = median_zd.minus(new_zd).l2Norm();

        float metric = kGeometryParameter * ns_minus_os + kShapeParameter * nz_minus_mz;

        if (chosen_node == NULL || metric < min_metric) {
          min_metric = metric;
          chosen_node = child_node;
        }
      }

      return chosen_node;
    }

    // Given a list of child data, choose two seeds for the splitting process in quadratic time
    static void pickSeedsQuadratic(const std::vector<NodeData>& split_data, int32* seed0, int32* seed1) {
      float THRESHOLD_PARAMETER = 0.8f;

      *seed0 = -1; *seed1 = -1;
      float max_waste = -FLT_MAX;
      float zernike_difference_for_max_waste_pair = -FLT_MAX;

      for(uint32 idx0 = 0; idx0 < split_data.size(); idx0++) {
            for(uint32 idx1 = idx0+1; idx1 < split_data.size(); idx1++) {
                BoundingSphere merged = split_data[idx0].bounding_sphere.merge(split_data[idx1].bounding_sphere);

                float waste = merged.volume() - split_data[idx0].bounding_sphere.volume() - split_data[idx1].bounding_sphere.volume();

                if (waste > max_waste) {
                    max_waste = waste;
                    *seed0 = idx0;
                    *seed1 = idx1;

                    zernike_difference_for_max_waste_pair = split_data[idx0].zernike_descriptor.
                                                            minus(split_data[idx1].zernike_descriptor).l2Norm();

                    zernike_difference_for_max_waste_pair /= split_data[idx0].zernike_descriptor.l2Norm();
                }
            }
      }

      for(uint32 idx0 = 0; idx0 < split_data.size(); idx0++) {
            for(uint32 idx1 = idx0+1; idx1 < split_data.size(); idx1++) {
                BoundingSphere merged = split_data[idx0].bounding_sphere.merge(split_data[idx1].bounding_sphere);

                float waste = merged.volume() - split_data[idx0].bounding_sphere.volume() - split_data[idx1].bounding_sphere.volume();

                float zernike_difference = split_data[idx0].zernike_descriptor.
                                                            minus(split_data[idx1].zernike_descriptor).l2Norm();

                zernike_difference /= split_data[idx0].zernike_descriptor.l2Norm();

                if (waste > max_waste*THRESHOLD_PARAMETER
                    && zernike_difference > zernike_difference_for_max_waste_pair)
                {
                    *seed0 = idx0;
                    *seed1 = idx1;
                    zernike_difference_for_max_waste_pair = zernike_difference;
                }
            }
      }

    }

    // Given list of split data and current group assignments as well as current group data, select the next child to be added and its group
    static void pickNextChild(std::vector<NodeData>& split_data, const SplitGroups& split_groups, const NodeData& group_data_0, const NodeData& group_data_1, int32* next_child, int32* selected_group)  {
        float max_metric = -1.0;
        *next_child = -1;
        *selected_group = -1;

	float normalizer = 0.0f;
	for(uint32 i = 0; i < split_data.size(); i++) {
            if (split_groups[i] != UnassignedGroup) continue;

            //Not doing any normalizing of these diffs for now, because it causes worse results. For example, if a sphere inflates from 2->10, it
            //is much worse than if it inflates from 1->5. But normalizing the two diffs makes them seems equally bad.
            BoundingSphere merged0 = group_data_0.bounding_sphere.merge(split_data[i].bounding_sphere);
            BoundingSphere merged1 = group_data_1.bounding_sphere.merge(split_data[i].bounding_sphere);

	    if (merged0.volume() > normalizer) normalizer = merged0.volume();
            if (merged1.volume() > normalizer) normalizer = merged1.volume();
	}

        for(uint32 i = 0; i < split_data.size(); i++) {
            if (split_groups[i] != UnassignedGroup) continue;

	    //Not doing any normalizing of these diffs for now, because it causes worse results. For example, if a sphere inflates from 2->10, it
	    //is much worse than if it inflates from 1->5. But normalizing the two diffs makes them seems equally bad.
            BoundingSphere merged0 = group_data_0.bounding_sphere.merge(split_data[i].bounding_sphere);
            BoundingSphere merged1 = group_data_1.bounding_sphere.merge(split_data[i].bounding_sphere);

            float diff0 = (merged0.volume() - group_data_0.bounding_sphere.volume())/normalizer;
            float diff1 = (merged1.volume() - group_data_1.bounding_sphere.volume())/normalizer;

            float zdiff0 = group_data_0.zernike_descriptor.minus(split_data[i].zernike_descriptor).l2Norm() ;
            float zdiff1 = group_data_1.zernike_descriptor.minus(split_data[i].zernike_descriptor).l2Norm() ;

            float metric0 = kShapeParameter * zdiff0 + kGeometryParameter * diff0;
            float metric1 = kShapeParameter * zdiff1 + kGeometryParameter * diff1;

            float metric = fabs(metric0-metric1);

            if (metric > max_metric) {
                max_metric = metric;
                *next_child = i;
                *selected_group = (metric0 < metric1) ? 0 : 1;
            }
        }
    }

    void verifyChild(const NodeData& child) const {
      BoundingSphereDataBase<SimulationTraits, SimilarMaxSphereData, CutNode>::verifyChild(child);

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

    Vector3 getBoundsCenter() const {
        return ThisBase::bounding_sphere.center();
    }
    Real getBoundsCenterBoundsRadius() const {
        return ThisBase::bounding_sphere.radius();
    }
    Real getBoundsMaxObjectSize() const {
        return mMaxRadius;
    }

    /** Gets the volume of this bounds of this region. */
    float volume() const {
      return getBounds().volume();
    }

    String getMesh() const {
      return mesh;
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
    ZernikeDescriptor zernike_descriptor;
    String mesh;
    DescriptorReader* descriptorReader;

};



// Cut notification utilities.
// Notify all cuts passing through the given node by calling the method with the
// CutNode and the given parameters.
template<typename SimulationTraits, typename NodeData, typename CutNode, typename F, typename P1>
void RTree_notify_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    F func, const P1& p1
)
{
    for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = node->cutNodesBegin(); cut_it != node->cutNodesEnd();) {
        CutNode* cutnode = cut_it->second;
        cut_it++; // Advance now to avoid invalidating iterator in callback
        func(cutnode, p1);
    }
}
template<typename SimulationTraits, typename NodeData, typename CutNode, typename F, typename P1, typename P2>
void RTree_notify_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    F func, const P1& p1, const P2& p2
)
{
    for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = node->cutNodesBegin(); cut_it != node->cutNodesEnd();) {
        CutNode* cutnode = cut_it->second;
        cut_it++; // Advance now to avoid invalidating iterator in callback
        func(cutnode, p1, p2);
    }
}
template<typename SimulationTraits, typename NodeData, typename CutNode, typename F, typename P1, typename P2, typename P3>
void RTree_notify_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    F func, const P1& p1, const P2& p2, const P3& p3
)
{
    for(typename CutNodeContainer<CutNode>::CutNodeListConstIterator cut_it = node->cutNodesBegin(); cut_it != node->cutNodesEnd();) {
        CutNode* cutnode = cut_it->second;
        cut_it++; // Advance now to avoid invalidating iterator in callback
        func(cutnode, p1, p2, p3);
    }
}


// Notify cuts *below* node with the given call. This is useful if you have an
// event at a node (e.g. the root) which is relevant to cuts that have refined
// past it. The cuts through the specified node *are not* notified.
template<typename SimulationTraits, typename NodeData, typename CutNode, typename F, typename P1>
void RTree_notify_descendant_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    F func, const P1& p1
)
{
    // Just follow one path until we hit the leaf nodes, notifying all cuts we
    // encounter along the way
    RTreeNode<SimulationTraits, NodeData, CutNode>* n = node;
    while(n != NULL && !n->objectChildren() && !n->empty()) {
        // Go all the way to the right so we get the "closest" nodes
        // to the event
        n = n->node( n->size()-1 );
        RTree_notify_cuts(n, func, p1);
    }
}
template<typename SimulationTraits, typename NodeData, typename CutNode, typename F, typename P1, typename P2>
void RTree_notify_descendant_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    F func, const P1& p1, const P2& p2
)
{
    // Just follow one path until we hit the leaf nodes, notifying all cuts we
    // encounter along the way
    RTreeNode<SimulationTraits, NodeData, CutNode>* n = node;
    while(n != NULL && !n->objectChildren() && !n->empty()) {
        // Go all the way to the right so we get the "closest" nodes
        // to the event
        n = n->node( n->size()-1 );
        RTree_notify_cuts(n, func, p1, p2);
    }
}
template<typename SimulationTraits, typename NodeData, typename CutNode, typename F, typename P1, typename P2, typename P3>
void RTree_notify_descendant_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    F func, const P1& p1, const P2& p2, const P3& p3
)
{
    // Just follow one path until we hit the leaf nodes, notifying all cuts we
    // encounter along the way
    RTreeNode<SimulationTraits, NodeData, CutNode>* n = node;
    while(n != NULL && !n->objectChildren() && !n->empty()) {
        // Go all the way to the right so we get the "closest" nodes
        // to the event
        n = n->node( n->size()-1 );
        RTree_notify_cuts(n, func, p1, p2, p3);
    }
}



template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_choose_leaf(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t)
{
    RTreeNode<SimulationTraits, NodeData, CutNode>* node = root;

    while(!node->objectChildren()) {
        RTreeNode<SimulationTraits, NodeData, CutNode>* min_increase_node = node->selectBestChildNode(obj_id, t);
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
    if (selected_group == 0) {
      group_data_0.mergeIn(split_data[next_child], split_data.size());
    }
    else {
      group_data_1.mergeIn(split_data[next_child], split_data.size());
    }

    return;
}


// NOTE: to support 2 versions, lifting cuts (possibly less efficient
// since you need to move cuts possibly very far up the tree, and
// definitely generates more event traffic) and not lifting cuts (more
// complicated, might end up being more expensive if there are a ton
// of cuts and they require a lot of work to rearrange), we split the
// node splitting code into 3 parts: preparation, the actual split
// (shared) and cleanup
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_split_node_prepare_before_split(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const typename SimulationTraits::TimeType& t);
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_split_node_generate_new_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node);
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_split_node_prepare(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t);
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_split_node_main(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t);
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_split_node_cleanup(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t);

// Splits a node, distributing its children between the two nodes
// (guaranteeing at least one empty spot in each node). Adds the new
// node as a child of the existing node's parent and returns the new
// node.
template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_split_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const typename SimulationTraits::TimeType& t)
{
    // We have a ton of steps here to accomodate the different
    // ordering between cut lifting and non-cut-lifting. Gross, but
    // better than having lots of duplicated code...
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    RTree_split_node_prepare_before_split<SimulationTraits, NodeData, CutNode>(node, t);
    RTreeNodeType* nn = RTree_split_node_generate_new_node<SimulationTraits, NodeData, CutNode>(node);
    RTree_split_node_prepare<SimulationTraits, NodeData, CutNode>(node, nn, t);
    RTree_split_node_main<SimulationTraits, NodeData, CutNode, ChildType, ChildOperations>(node, nn, t);
    RTree_split_node_cleanup<SimulationTraits, NodeData, CutNode, ChildType, ChildOperations>(node, nn, t);
    return nn;
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_split_node_generate_new_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    assert(!node->parent()->full());
    RTreeNodeType* nn = new RTreeNodeType(node->owner());
    nn->objectChildren(node->objectChildren());
    // We need to be careful about insertion of the new node for the
    // case when we're not lifting cuts. If the node we're splitting
    // isn't the last node in the parent, we could screw up the order
    // of the cut if we just insert it at the end since the nodeSplit
    // callback just tells it that we're splitting that node, and that
    // code assumes (since there's no other reasonable approach) that
    // the new node is next to the old one.
    //
    // So, to handle this properly, we need to insert right after the
    // node we split into two. We specify this to the insert call,
    // which shifts the other elements over.
    node->parent()->insert(nn, node);

    return nn;
}

template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_split_node_main(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    LocationServiceCache<SimulationTraits>* loc = node->loc();

    ChildOperations child_ops;

    // collect the info for the children
    std::vector<ChildType> split_children;
    std::vector<NodeData> split_data;
    SplitGroups split_groups;

    // add all the children to the split vectors
    for(int i = 0; i < node->size(); i++) {
        split_children.push_back( child_ops.childData(node, i) );
        split_data.push_back( node->childData(i,t) );
        split_groups.push_back(UnassignedGroup);
    }

    // find the initial seeds
    NodeData group_data_0, group_data_1;
    RTree_quadratic_pick_seeds(split_data, split_groups, group_data_0, group_data_1);

    // group the remaining ones
    for(uint32 i = 0; i < split_children.size()-2; i++) {
        RTree_pick_next_child(split_data, split_groups, group_data_0, group_data_1);
    }

    // reparent data that needs to be moved into the new node
    // the new node nn has already been inserted so we are guaranteed
    // correct structure/ordering
    for(uint32 i = 0; i < split_children.size(); i++) {
        // We only need to reparent into the new node
        if (split_groups[i] == 0) continue;
        child_ops.reparent( node, nn, split_children[i], t );
    }
}

#ifdef LIBPROX_LIFT_CUTS

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_split_node_prepare_before_split(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    /** Since we're splitting node, we need to lift any cuts up to its
     * parent. If its the root, just lift to the node itself and the
     * notification of splits should take care of getting the cut right.
     */
    RTreeNodeType* parent = node->parent();
    if (parent)
        RTree_lift_cut_nodes_from_tree(node, parent);
    else
        RTree_lift_cut_nodes_from_tree(node, node);
    RTree_verify_no_cut_nodes_in_tree(node);
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_split_node_prepare(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t)
{
}

template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_split_node_cleanup(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    const typename RTreeNodeType::Callbacks& cb = node->callbacks();

    // Notify the cuts of the split so it can be updated. Either all have been
    // lifted up to this node if its the root or there are none because they
    // have been lifted to the parent.  There should be no cuts in the children
    // nodes.
    if (cb.nodeSplit)
        RTree_notify_cuts(node, cb.nodeSplit, node, nn);
    // No replicatedNodeSplit notification because we lifted cuts -- there's
    // nothing in the descendant nodes to notify
}

#else // LIBPROX_LIFT_CUTS

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_split_node_prepare_before_split(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const typename SimulationTraits::TimeType& t)
{
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_split_node_prepare(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    const typename RTreeNodeType::Callbacks& cb = node->callbacks();

    // Nothing to prepare since lifting cuts isn't allowed. However,
    // we do notify cuts immediately so that when nodes/objects are
    // reparented, the new node will already be available (addition
    // added so we don't try to add/reparent to nodes that haven't
    // been reported yet). We also notify descendant cuts now.
    if (cb.nodeSplit) {
        assert(cb.nodeSplit && cb.reorderCut);
        // If we saw a split, we need to deal with cleaning up
        // cuts. First, we need to get new cut nodes into cuts for the
        // newly generated node. We only need to insert these for cuts
        // going through the node that we split into two. Cuts that pass
        // through the tree lower down should have already been taken
        // care of and should alrady span the entire width of the tree.
        RTree_notify_cuts(node, cb.nodeSplit, node, nn);
        // And notify cuts through descendant nodes of the
        // split. The split should have taken care of making sure there
        // are cut nodes through both subtrees, so we can just send the
        // notification to one of them.
        if (cb.replicatedNodeSplit)
            RTree_notify_descendant_cuts(node, cb.replicatedNodeSplit, node, nn);
    }
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

        if (n->objectChildren()) break;
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

        if (n->objectChildren()) break;
        assert(!n->empty());
        n = n->node( n->size()-1 );
    }
}

template<typename SimulationTraits, typename NodeData, typename CutNode, typename ChildType, typename ChildOperations>
void RTree_split_node_cleanup(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    const typename RTreeNodeType::Callbacks& cb = node->callbacks();

    // A contiguous range within a cut. Used to record contiguous segments of a
    // cut that are being moved.
    typedef typename CutNode::RangeType CutNodeRange;
    typedef typename CutNode::CutType Cut;
    typedef std::vector<typename CutNode::RangeType> CutRangeVector;
    typedef std::map<typename CutNode::CutType*, CutRangeVector> CutRangeMap;

    if (cb.nodeSplit) {
        assert(cb.nodeSplit && cb.reorderCut);
        // Cuts were already notified of the split, just need to deal
        // with reordering.

        // Now we should have all the parts of the cut that should be in
        // the tree. Now we need to patch up the order of things since
        // the split rearranged cuts. This rearrangement can't just
        // apply to cuts passing through these nodes: any cut lower in
        // the tree could have become jumbled. However, we only need to
        // deal with "blocks" of the cuts under each child since we only
        // jumble things around one layer down (the children).
        if (!node->objectChildren()) {
            // Only need to do (and only can do) rearrangement if we're
            // not at leaves.
            assert(!nn->objectChildren());

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

    if (cb.nodeSplitFinished) {
        // Let cuts passing through this node know that splitting
        // these nodes finished, which allows for some final
        // processing. This is used to ensure we keep the split nodes
        // at the same "level" when they are leaf nodes -- if the cut
        // is through node but object results are returned, then we
        // want to refine the new node to objects as well.
        RTree_notify_cuts(node, cb.nodeSplitFinished, node, nn);
    }
}


// Walks up the tree, dealing with cascading insertions due to an inserted
// object and recomputing aggregate info as necessary. Returns the new root node
// (or existing one if it remains the same)
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_adjust_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* L,
    RTreeNode<SimulationTraits, NodeData, CutNode>* LL,
    const typename SimulationTraits::TimeType& t)
 {
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    assert(L->objectChildren());
    RTreeNode<SimulationTraits, NodeData, CutNode>* node = L;
    RTreeNode<SimulationTraits, NodeData, CutNode>* nn = LL;

    while(true) { // loop until root, enter for root as well so we recompute its bounds
        RTreeNode<SimulationTraits, NodeData, CutNode>* parent = node->parent();
        const typename RTreeNodeType::Callbacks& cb = node->callbacks();

        // FIXME this is inefficient
        node->recomputeData(t);

        RTreeNode<SimulationTraits, NodeData, CutNode>* pp = NULL;
        if (nn != NULL) {

        }

        if (parent == NULL) break;

        node = parent;
        nn = pp;
    }

    return node;
}

#endif


// Helper for RTree_insert_object_at_node that finds the nodes that
// need to be split. We need to compute this because we need to work
// top down and need to know the set of nodes that should be split.
//
// The output is a list of nodes that need to be split. It can be
// empty if the inserted object fits in the leaf node. The top split
// (the last in the list) may be the root, in which case a new root
// needs to be put in place above the existing root. Note that this
// will include the leaf node that needs to be split for *objects* not
// children nodes.
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_build_split_nodes_list(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_node,
    std::vector< RTreeNode<SimulationTraits, NodeData, CutNode>* >& split_nodes_out)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    RTreeNodeType* node = NULL;
    RTreeNodeType* split_parent = leaf_node;

    // Just keep stepping up while adding another node to the parent
    // (which is caused by the split) causes the node to hit
    // capacity.
    while(split_parent != NULL && split_parent->full()) {
        split_nodes_out.push_back(split_parent);

        node = split_parent;
        split_parent = split_parent->parent();
    }
}

// Helper function. Recompute the bounds of all nodes on the path from
// the given node to the root, e.g. because of insertion or deletion.
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_recompute_bounds_from_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_node,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    RTreeNodeType* node = leaf_node;
    while(node != NULL) {
        node->recomputeData(t);
        node = node->parent();
    }
}

// "Prepends" a new root to the tree, thereby adding a level to the
// tree. This utility also makes sure cuts are also notified of the
// change.
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_prepend_new_root(
    RTreeNode<SimulationTraits, NodeData, CutNode>* new_root,
    RTreeNode<SimulationTraits, NodeData, CutNode>* root)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    assert(new_root->empty());
    new_root->objectChildren(false);
    new_root->insert(root);

    // All replicated tree cuts will want to know that there's a new root node
    if (root->callbacks().replicatedRootCreated)
        RTree_notify_descendant_cuts(new_root, root->callbacks().replicatedRootCreated, root, new_root);
}

// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_insert_object_at_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_node,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef std::vector<RTreeNodeType*> RTreeNodeList;

    // This function needs to make sure this is actually a leaf because it can
    // be used to insert objects for a regular tree or from replication. In
    // replication we may not have marked it as a leaf properly yet (because
    // if this is the first child, this is the first time we would know that it
    // is a leaf). Sanity check and then make sure the setting is right.
    assert( leaf_node->objectChildren() || (leaf_node->empty() && leaf_node->replicated()) );
    // And once we know it's safe, make sure we have the setting right
    leaf_node->objectChildren(true);

    // We can't just start inserting here because we could cause
    // overflow. For tree replication to work simply, we need to
    // generate a sequence of events which can be streamed in and
    // applied immediately. Working bottom up would generate a
    // sequence of splits where we would initially add internal with
    // no parent and then add a parent later.
    //
    // Instead, we'll figure out where we're going to need to perform
    // our last split (possibly at the root) and work our way down. At
    // least for now, since we only split when things get full, we
    // know that whereever we have a split, we'll need to split at
    // each level down the tree, until we hit the leaves.
    RTreeNodeList nodes_to_split;
    RTree_build_split_nodes_list(leaf_node, nodes_to_split);
    RTreeNodeType* root = leaf_node->root();

    // If we hit the root, we need to start by creating the new root
    // and putting the old root as a child.
    if (!nodes_to_split.empty() && nodes_to_split.back() == root) {
        RTreeNodeType* new_root = new RTreeNodeType(root->owner());
        RTree_prepend_new_root(new_root, root);
        root = new_root;
    }

    // Now we need to process splits. The list is ordered bottom up,
    // so we need to work backwards. The first entry, if we have one,
    // is the leaf node (dealing with objects, not nodes), so we skip
    // it, dealing with it last.
    RTreeNodeList nodes_to_update_bounds;
    for(int32 node_to_split_idx = nodes_to_split.size()-1; node_to_split_idx > 0; node_to_split_idx--) {
        RTreeNodeType* split_node = nodes_to_split[node_to_split_idx];
        // We could get the node as the return value, but we don't
        // actually care about it -- it has already been inserted into
        // the tree.
        RTreeNodeType* new_node = RTree_split_node<SimulationTraits, NodeData, CutNode, RTreeNode<SimulationTraits, NodeData, CutNode>*, typename RTreeNode<SimulationTraits, NodeData, CutNode>::NodeChildOperations>(split_node, t);
        nodes_to_update_bounds.push_back(split_node);
        nodes_to_update_bounds.push_back(new_node);
    }

    // The above loop deals with nodes-with-node-children splits. We
    // need to deal with a possible final node split at the leaf node.
    if (leaf_node->full()) {
        RTreeNodeType* new_leaf_node = RTree_split_node<SimulationTraits, NodeData, CutNode, typename LocationServiceCache<SimulationTraits>::Iterator, typename RTreeNode<SimulationTraits, NodeData, CutNode>::ObjectChildOperations>(leaf_node, t);
        // Make sure the bounds are updated for *just* these two nodes -- we
        // need to make sure they are correct so the selection step we do uses
        // correct information. Their ancestors will be updated along with all
        // other nodes when we actually finish the update.
        leaf_node->recomputeData(t);
        new_leaf_node->recomputeData(t);

        // Since we split the leaf we were going to insert into, we
        // need to now choose where to put it. We *must* choose
        // between only these two or we could end up just splitting
        // things again.
        leaf_node = RTreeNodeType::selectBestChildNodeFromPair(leaf_node, new_leaf_node, obj_id, t);
    }

    // After this point, only leaf_node and its ancestors are modified, one
    // of the two nodes we just split doesn't need to get into the update list.
    nodes_to_update_bounds.push_back(leaf_node);

    // The final step, once we've reached the bottom of the tree, is
    // to actually insert the object. The previous step should ensure
    // that the leaf node isn't full.
    assert(!leaf_node->full());
    leaf_node->insert(obj_id, t);

    // Top down splitting and insertion isn't going to get bounds right -- the
    // splitting process isn't updating them, updating from the new leaf doesn't
    // handle multiple splits properly, and top-down splits means we can't
    // correctly update them as we go. Instead, we've collected all the
    // split/modified nodes, now we just need to update them. We could probably
    // save effort by tracking which ones we've already updated (or have a
    // method to update them all at the same time, walking up the tree
    // gradually), but this is relatively cheap anyway...
    for(typename RTreeNodeList::iterator update_it = nodes_to_update_bounds.begin(); update_it != nodes_to_update_bounds.end(); update_it++)
        RTree_recompute_bounds_from_node(*update_it, t);

    return root;
}

// Inserts a new object into the tree, updating any nodes as necessary. Returns the new root node.
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_insert_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    RTreeNodeType* leaf_node = RTree_choose_leaf(root, obj_id, t);
    // In this case, it better already be marked as a leaf node
    assert(leaf_node->objectChildren());
    return RTree_insert_object_at_node(leaf_node, obj_id, t);
}



template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_create_new_node(
    RTree<SimulationTraits, NodeData, CutNode>* owner,
    const typename LocationServiceCache<SimulationTraits>::Iterator& node,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    return new RTreeNodeType(owner, node, t);
}



// Inserts a new, empty node into the tree with the given node as it's parent,
// updating any ancestors nodes as necessary. Returns the new node
// (unlike insert object, it should never be possible for this to
// result in a new root since it assumes space is available for the node).
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_insert_new_node_at_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* new_node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* parent_node)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // Sanity check the parent
    assert(!parent_node->full());
    // Should be either marked as not a leaf or should be empty so we can change it
    assert(!parent_node->objectChildren() || parent_node->empty());
    bool was_empty = parent_node->empty();
    // And once we know it's safe, make sure we have the setting right
    parent_node->objectChildren(false);

    parent_node->insert(new_node);

    // No cleanup -- cannot cause overflow and splitting

    // Let any cuts that are below this one know that a new node was
    // added. If the node was empty, there couldn't have been any
    // nodes under it, so we can skip this
    if (new_node->callbacks().nodeAddedAboveCut && !was_empty) {
        // We're careful here to notify with the right cut nodes, the
        // ones "closest" to the event where we'll need to insert the
        // new node in the cut. First we find the closest node
        RTreeNodeType* closest_node = parent_node->node(parent_node->size()-2);
        // Notify its cuts
        RTree_notify_cuts(closest_node, new_node->callbacks().nodeAddedAboveCut, new_node);
        // and then cuts in its descendants
        RTree_notify_descendant_cuts(closest_node, new_node->callbacks().nodeAddedAboveCut, new_node);
    }
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
int32 RTree_count(RTreeNode<SimulationTraits, NodeData, CutNode>* root) {
    if (root->objectChildren()) return root->size();

    int32 result = 0;
    for(int i = 0; i < root->size(); i++)
        result += RTree_count(root->node(i));
    return result;
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_verify_constraints(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const typename SimulationTraits::TimeType& t)
{
#ifdef PROXDEBUG
    for(int i = 0; i < root->size(); i++) {
        if(!root->objectChildren()) {
            assert(root->node(i)->parent() == root);
        }
        root->data().verifyChild( root->childData(i, t) );
    }
    if (!root->objectChildren()) {
        for(int i = 0; i < root->size(); i++)
            RTree_verify_constraints(root->node(i), t);
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
    RTreeNode<SimulationTraits, NodeData, CutNode>* to_node)
{
    // Only notify cuts if we're moving *above* this node, not if this node is
    // both the starting point for recursion and the destination node
    if (from_node != to_node) {
        // Notify cuts to lift up to the right node
        // NOTE: We use this approach since the callback likely adjusts the cut
        // node list in from_node
        while(!from_node->cutNodesEmpty())
            from_node->callbacks().liftCut(from_node->cutNodesBegin()->second, to_node);
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
    RTreeNode<SimulationTraits, NodeData, CutNode>* to_node)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename RTreeNodeType::Index Index;

    RTree_lift_cut_nodes(from_node, to_node);

    // And recurse
    if (!from_node->objectChildren()) {
        for(Index idx = 0; idx < from_node->size(); idx++) {
            RTree_lift_cut_nodes_from_tree(from_node->node(idx), to_node);
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
    if (!node->objectChildren())
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
    const typename SimulationTraits::TimeType& t)
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
        RTree_lift_cut_nodes_from_tree(highest_removed, parent);
        RTree_verify_no_cut_nodes_in_tree(highest_removed);

        // Then, remove nodes
        std::queue<RTreeNodeType*> removedNodes;
        parent->erase(highest_removed);
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
            if (removed->objectChildren()) {
                for(Index idx = 0; idx < removed->size(); idx++)
                    root = RTree_insert_object(root, removed->object(idx).object, t);
            }
            else {
                while(removed->size()) {
                    RTreeNodeType* child_removed = removed->node(0);
                    removedNodes.push(child_removed);
                    removed->erase(child_removed);
                }
            }
            removed->destroy();
        }
    }


    // After removal, there's a chance that the root node ended up with no
    // elements, in which case it should be marked as a leaf node
    if (root->size() == 0)
        root->objectChildren(true);

    // Perform recomputation of node data
    RTree_recompute_bounds_from_node(recompute_start, t);

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
    const typename SimulationTraits::TimeType& t)
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
            all_nodes = all_nodes && !parent->node(ci)->objectChildren();
            all_objects = all_objects && parent->node(ci)->objectChildren();
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
                RTree_lift_cut_nodes(parent->node(ci), parent);
            }

            // Remove the children nodes, keeping track of them
            std::vector<RTreeNodeType*> child_nodes;
            while(!parent->empty()) {
                child_nodes.push_back(parent->node( parent->size()-1 ));
                parent->erase(child_nodes.back());
            }

            // Make sure the parent is the right type of node. It
            // should be empty now. We also explicitly ask for clear()
            // because this resets the bounds data, which will then be
            // updated as we insert the objects
            assert(parent->empty());
            parent->clear();
            parent->objectChildren(all_objects);

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
                    if (child_nodes[ci]->objectChildren()) {
                        LocCacheIterator gchild_object = child_nodes[ci]->object(0).object;
                        child_nodes[ci]->erase(gchild_object, true);
                        parent->insert(gchild_object, t);
                    }
                    else {
                        RTreeNodeType* gchild_node = child_nodes[ci]->node(0);
                        child_nodes[ci]->erase(gchild_node);
                        parent->insert(gchild_node);
                    }
                }
                child_nodes[ci]->destroy();
            }

            // Force recomputation of bounds as we move up the rest of
            // the tree.
            dirty_bounds = true;
        }
        else if (n->empty()) {
            // Even if we couldn't merge all siblings into the grandparent, this
            // node might still be empty.

            // Remove from parent and destroy. Split erase/destroy so
            // we can validate cuts in cut callback
            // Get cuts out of this node.
            if (parent->size() == 1) {
                RTree_lift_cut_nodes(n, parent);
                parent->erase(n);
            }
            else {
                parent->erase(n);
                RTree_notify_cuts(n, n->callbacks().nodeWithCutRemoved, n);
            }
            n->destroy();

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
            parent->recomputeData(t);
    }

    // We need to return the new root. The loop above should have
    // taken us there, so the current node should be the root.
    return n;
}

#endif

/* Updates the object in the given tree. Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_update_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const typename SimulationTraits::ObjectIDType& obj_id,
    const typename SimulationTraits::TimeType& t)
{
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_with_obj = root->callbacks().getObjectLeaf(obj_id);
    if (leaf_with_obj == NULL)
        return root;

    RTree_recompute_bounds_from_node(leaf_with_obj, t);

    return root;
}

/* Updates the node in the given tree, and applies updates up the tree. Should
 * only be used for replicated trees. Since this guarantees updates propagate up
 * the tree, it only applies updates at nodes that do not have children (not
 * equivalent to leaf nodes since you can have a replicated non-leaf node with
 * no children replicated).
 */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_update_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const typename LocationServiceCache<SimulationTraits>::Iterator& node_locit,
    const typename SimulationTraits::TimeType& t)
{
    if (!node->empty()) return;

    // Need to force the update of , and only call recomputeData from the parent
    // up since this node doesn't have any children.
    node->updateReplicatedNodeData(node_locit, t);
    RTree_recompute_bounds_from_node(node->parent(), t);
}

/* Updates objects in the tree with new positions. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_update_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const typename SimulationTraits::TimeType& t)
{
    if (!root->objectChildren()) {
        for(int i = 0; i < root->size(); i++) {
            // FIXME set update node
            RTree_update_tree(root->node(i), t);
        }
    }

    root->recomputeData(t);

    return root;
}

/* Reparents a node and notifies affected cuts. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_reparent_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    RTreeNode<SimulationTraits, NodeData, CutNode>* new_parent,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    RTreeNodeType* old_parent = node->parent();
    assert(old_parent != NULL);
    // Currently this should only be possible if there's a split and
    // the new parent and old parent have the same grandparent
    assert(old_parent->parent() == new_parent->parent());

    // Sanity check the new parent
    assert(!new_parent->full());
    // The new parent might be a fresh node and we might need to make
    // sure it's set to the right type.
    // Should be either marked as not a leaf or should be empty so we can change it
    assert(!new_parent->objectChildren() || new_parent->empty());
    new_parent->objectChildren(false);

    // Perform the actual reparenting. This also triggers callbacks to
    // any cuts through this replicated tree that might be affected.
    new_parent->reparent(node, old_parent);
}

template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_collect_cuts(
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    std::tr1::unordered_set<typename CutNode::CutType*>* cuts)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename CutNode::CutType Cut;

    for(typename RTreeNodeType::CutNodeListIterator it = node->cutNodesBegin(); it != node->cutNodesEnd(); it++) {
        Cut* cut = it->first;
        cuts->insert(cut);
    }

    if (node->objectChildren())
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
    const typename SimulationTraits::TimeType& t)
{
    fprintf(fout, "{ ");

    // Report this nodes volume
    float this_volume = root->data().volume();
    fprintf(fout, " \"volume\" : %f, \"children\" : [ ", this_volume);
    // Recurse
    if (root->objectChildren()) {
        for(int i = 0; i < root->size(); i++) {
            if (i > 0) fprintf(fout, ", ");
            fprintf(fout, "{ \"volume\" : %f }", root->childData(i, t).volume());
        }
    }
    else {
        for(int i = 0; i < root->size(); i++) {
            if (i > 0) fprintf(fout, ", ");
            RTree_report_bounds(fout, root->node(i), t);
        }
    }

    fprintf(fout, " ] }");
}

/* Cleanup performed after deletion of an object/node from a node. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_post_deletion_cleanup(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    RTreeNode<SimulationTraits, NodeData, CutNode>* clean_start,
    const typename SimulationTraits::TimeType& t)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    RTreeNodeType* new_root = RTree_condense_tree(clean_start, t);

    // We might need to shorten the tree if the root is left with only one child.
    if (!root->objectChildren() && root->size() == 1) {
        new_root = root->node(0);
        // Notify cuts so they can refine to the new root.
        if (root->callbacks().rootReplaced)
            RTree_notify_cuts(root, root->callbacks().rootReplaced, root, new_root);
        // Cuts through descendant nodes want to know that the root was destroyed
        if (root->callbacks().replicatedRootDestroyed)
            RTree_notify_descendant_cuts(root, root->callbacks().replicatedRootDestroyed, root, new_root);

        new_root->parent(NULL);
        root->destroy();
    }
    return new_root;
}

/* Deletes the object from the given tree.  Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_delete_object(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    const typename LocationServiceCache<SimulationTraits>::Iterator& obj_id,
    const typename SimulationTraits::TimeType& t,
    bool temporary)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;

    ObjectIDType real_obj_id = root->loc()->iteratorID(obj_id);
    RTreeNode<SimulationTraits, NodeData, CutNode>* leaf_with_obj =
        root->callbacks().getObjectLeaf(real_obj_id);
    if (leaf_with_obj == NULL) {
        return root;
    }

    // Notify any cuts that the object is leaving
    assert(leaf_with_obj->objectChildren());
    leaf_with_obj->erase(obj_id, temporary);

    // If the tree is replicated, we don't really want to do anything else, but
    // if it's our own tree then we want to cleanup nodes if we don't need them
    // anymore.
    if (root->replicated())
        return root;
    else
        return RTree_post_deletion_cleanup(root, leaf_with_obj, t);
}


/* Deletes the object from the given tree.  Returns the new root. */
template<typename SimulationTraits, typename NodeData, typename CutNode>
RTreeNode<SimulationTraits, NodeData, CutNode>* RTree_delete_node(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    RTreeNode<SimulationTraits, NodeData, CutNode>* node,
    const typename SimulationTraits::TimeType& t,
    bool temporary)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;

    // Removing only makes sense if we're replicating trees.
    assert(root != NULL && root->replicated());

    // We have to deal with some weird cases like removing the root node to
    // shorten the tree:
    RTreeNodeType* parent = node->parent();
    if (parent == NULL) {
        assert(node == root);
        assert(node->empty() || (!node->objectChildren() && node->size() == 1));
        RTreeNodeType* new_root = (node->empty() ? NULL : node->node(0));
        // Notify cuts so they can refine to the new root.
        if (root->callbacks().rootReplaced)
            RTree_notify_cuts(node, node->callbacks().rootReplaced, node, new_root);

        if (new_root) new_root->parent(NULL);
        node->destroy();

        return new_root;
    }

    // Otherwise, we should be at the bottom of the tree (not necessarily a leaf
    // since it's a partially replicated tree).
    assert(!parent->objectChildren());
    assert(node->empty());
#ifdef LIBPROX_LIFT_CUTS
    // And we need to get cuts out of the way. lift_cut_nodes should be
    // sufficient (rather than lift_cut_nodes_from_tree) since there's nowhere
    // deeper to go
    RTree_lift_cut_nodes(node, parent);
    // And then erase the node and destroy it
    parent->erase(node);
    node->destroy();
#else
    // Or, for non-cut-lifting, we need to tell the cuts to clean this node out
    // of their cut. If we're removing the last child of the parent, this
    // degenerates to lifting anyway and we reuse the logic. Otherwise, we do a
    // simple removal.
    // We need to also erase the node and destroy it. Split
    // erase/destroy so we can validate cuts in cut callback
    if (parent->size() == 1) {
        RTree_lift_cut_nodes(node, parent);
        parent->erase(node);
    }
    else {
        parent->erase(node);
        RTree_notify_cuts(node, node->callbacks().nodeWithCutRemoved, node);
    }
    node->destroy();
#endif

    // Removing nodes only makes sense when replicating trees. We don't want to
    // do any additional cleanup in that case, so we can just return the old
    // root.
    return root;
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

    if (root->objectChildren()) {
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
    RTreeNode<SimulationTraits, NodeData, CutNode>* root)
{
    typedef RTreeNode<SimulationTraits, NodeData, CutNode> RTreeNodeType;
    typedef typename SimulationTraits::ObjectIDType ObjectIDType;

    if (!root->objectChildren()) {
        while(root->size()) {
            RTreeNodeType* child = root->erasePop();
            RTree_destroy_tree(child);
        }
    }

    root->destroy();
}



// NOTE: This is debugging code and only works if your ObjectID type is ostream
// compatible. Don't check calls to this in, but it can be handy for debugging.
template<typename SimulationTraits, typename NodeData, typename CutNode>
void RTree_draw_tree(
    RTreeNode<SimulationTraits, NodeData, CutNode>* root,
    int32 indent = 0)
{

    if (root == NULL) {
        std::cout << "(NULL Tree)" << std::endl;
        return;
    }

    for(int i = 0; i < indent; i++) std::cout << " ";
    std::cout << root->aggregateID() << " with " << root->cutNodesSize() << " cuts" << std::endl;

    if (root->objectChildren()) {
        for(int ci = 0; ci < root->size(); ci++) {
            for(int i = 0; i < indent+1; i++) std::cout << " ";
            std::cout << root->loc()->iteratorID(root->object(ci).object) << std::endl;
        }
        //std::cout << "(" << root->size() << " object children)" << std::endl;
    }
    else {
        for(int ci = 0; ci < root->size(); ci++)
            RTree_draw_tree(root->node(ci), indent+1);
    }
}

} // namespace Prox

#endif //_PROX_RTREE_CORE_HPP_
