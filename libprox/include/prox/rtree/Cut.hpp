// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_RTREE_CUT_HPP_
#define _PROX_RTREE_CUT_HPP_

#include "CutNode.hpp"

namespace Prox {

template <typename SimulationTraits, typename QueryHandlerTypeT, typename NodeDataTypeT, typename CutTypeT, typename CutNodeTypeT>
class CutBase {
public:
    typedef LocationServiceCache<SimulationTraits> LocationServiceCacheType;
    typedef typename LocationServiceCacheType::Iterator LocCacheIterator;

    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef typename SimulationTraits::ObjectIDNullType ObjectIDNull;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::realType Real;

    typedef AggregateListener<SimulationTraits> AggregateListenerType;

    typedef QueryEvent<SimulationTraits> QueryEventType;

    typedef QueryHandlerTypeT QueryHandlerType;
    typedef typename QueryHandlerType::QueryType QueryType;
    typedef NodeDataTypeT NodeDataType;
    typedef CutTypeT CutType;
    typedef CutNodeTypeT CutNodeType;

    typedef std::vector<typename CutNodeType::RangeType> CutRangeVector;

    typedef typename Prox::RTree<SimulationTraits, NodeDataType, CutNodeType>::RTreeNodeType RTreeNodeType;

public:
    // Gets the this pointer in the native Cut type, i.e. the real type of
    // this class rather than as this base class.
    CutType* getNativeThis() {
        return static_cast<CutType*>(this);
    }
    const CutType* getNativeThis() const {
        return static_cast<const CutType*>(this);
    }

    // NOTE that this class is a bit weird. It doesn't use virtual functions but
    // requires that a class that inherits from it has a certain set of methods:
    //  bool withAggregates() const; // Whether to use aggregates
    //  AggregateListenerType* aggregateListener();
    //  LocationServiceCacheType* locCache();
    //  const LocationServiceCacheType* locCache() const;
    //  const Time& curTime() const;
    //  RTreeNodeType* rootRTreeNode();
    //  bool rebuilding() const;
    //  void addResult(const ObjectID& objid);
    //  void removeResult(const ObjectID& objid);
    //  bool inResults(const ObjectID& objid) const;
    //  bool resultsSize() const;
    //    For insertion/deletion, to decide whether to add to the results
    //  bool satisfiesQuery(RTreeNodeType* node, LocCacheIterator objit, int objidx) const;
    // And these are helpers that make accessing these convenient within this
    // class.
    bool usesAggregates() const {
        return getNativeThis()->withAggregates();
    }
    AggregateListenerType* getAggregateListener() {
        return getNativeThis()->aggregateListener();
    }
    LocationServiceCacheType* getLocCache() const {
        return getNativeThis()->locCache();
    }
    const Time& getCurTime() const {
        return getNativeThis()->curTime();
    }
    RTreeNodeType* getRootRTreeNode() {
        return getNativeThis()->rootRTreeNode();
    }
    bool isRebuilding() const {
        return getNativeThis()->rebuilding();
    }
    void addToResults(const ObjectID& objid) {
        getNativeThis()->addResult(objid);
    }
    size_t removeFromResults(const ObjectID& objid) {
        return getNativeThis()->removeResult(objid);
    }
    bool isInResults(const ObjectID& objid) const {
        return getNativeThis()->inResults(objid);
    }
    int getResultsSize() const {
        return getNativeThis()->resultsSize();
    }
    bool checkSatisfiesQuery(RTreeNodeType* node, LocCacheIterator objit, int objidx) const {
        return getNativeThis()->satisfiesQuery(node, objit, objidx);
    }

    int cutSize() const {
        assert(length == (int)nodes.size());
        return length;
    };

    void handleRootReplaced(CutNodeType* cnode, RTreeNodeType* orig_root, RTreeNodeType* new_root) {
        // The old root was replaced by the new root because the tree is
        // getting smaller.  We just need to shift our cut down to the new
        // node.
        // FIXME linear search could be avoided by storing iterators
        CutNodeListIterator it = std::find(nodes.begin(), nodes.end(), cnode);
        if (usesAggregates()) {
            QueryEventType evt;
            it = replaceParentWithChildren(it, &evt);
            events.push_back(evt);
        }
        else {
            it = replaceParentWithChildren(it, NULL);
        }
    }

    // Handle a split of orig_node into orig_node and new_node. cnode is the
    // CutNode that was (and remains) at orig_node.
    void handleSplit(CutNodeType* cnode, RTreeNodeType* orig_node, RTreeNodeType* new_node) {
        // Split into two parts. The first part always has to happen -- it's
        // just making sure the cut crosses the tree since a new node has been
        // inserted.
        //
        // The second part handles the result set changes. For example, if you
        // aren't using aggregates, then maybe nothing has to happen -- just
        // having the new node in the cut will force it to consider things
        // properly in the future. If you *are* using aggregates, then you might
        // care that a) there's a new node and b) that children have been
        // rearranged, are split across 2 nodes, etc. You might need to add the
        // new node as an aggregate, or you might need to add the new child that
        // was added (for a leaf object), e.g. if you had the cut through the
        // node A, but all its children were in the result, then when split to A
        // and B, at least one child (the new one) would have been left out of
        // the result incorrectly.

        // By default we just add the cut node.
        handleSplitAddCutNode(cnode, orig_node, new_node);
        handleSplitResolveAdditions(cnode, orig_node, new_node);
    }

    // Helper that does the addition of the cut node.
    void handleSplitAddCutNode(CutNodeType* cnode, RTreeNodeType* orig_node, RTreeNodeType* new_node) {
        // Add a new CutNode to new_node and insert it in our cut list.
        // Future updates will take care of any additional changes (push up
        // or down) that still need to be applied to the tree.

        // FIXME we could avoid this linear search by storing iterators in CutNode
        CutNodeListIterator orig_list_it = std::find(nodes.begin(), nodes.end(), cnode);
        assert(orig_list_it != nodes.end());
        CutNodeListIterator after_orig_list_it = orig_list_it; after_orig_list_it++;

        CutNodeType* new_cnode = new CutNodeType(parent, getNativeThis(), new_node, getAggregateListener());
        nodes.insert(after_orig_list_it, new_cnode);
        length++;

        // Mid-operation, no validation
    }

    // After a cut node has been added because a new RTreeNode was added, this
    // handles adding/removing objects or aggregates from the result set based
    // on which were previously in the result set. Either the new node needs to
    // be added, or the additional child that caused the split needs to be
    // added.
    void handleSplitResolveAdditions(CutNodeType* cnode, RTreeNodeType* orig_node, RTreeNodeType* new_node) {
        // We only care about this if we are using aggregates. Otherwise at
        // worst we miss some stuff until the next reevaluation.
        if (!usesAggregates()) return;
        // The split can result in one of two cases:
        // In both cases, we should be generating an event of some kind.
        QueryEventType evt;
        // If the original node is in the result set, then none of the
        // children were in the result set. This means we can just add the
        // new node to the result set.
        if (isInResults(orig_node->aggregateID())) {
            evt.additions().push_back( typename QueryEventType::Addition(new_node, QueryEventType::Imposter) );
            addToResults(new_node->aggregateID());
        }
        else {
            // Otherwise, the children must be in the result set. In that
            // case, the new object (and only the new object) isn't in the
            // result set yet.
            // This should only happen if this is a leaf
            assert(orig_node->leaf());
            assert(new_node->leaf());
            // And we don't know where we inserted the object, so just cycle
            // through all of both nodes looking for the missing item.
            // TODO(ewencp). Currently, notifications are sent during
            // addition/removals for cuts with these results. Since this cut
            // went through the original, we'll actually have everything in
            // orig_node (everything was removed from orig_node, then those
            // items were added back) and we'll be missing everything from
            // new_node. We shouldn't generate these additions/removals. This
            // code, and when used with the commented assertion below, should
            // handle that case properly once we stop generating those events
            // when we're splitting nodes while keeping cuts in place. Currently
            // it works without the assertion since the end goal is to just add
            // all the children as results anyway.
            int32 nadded = 0;
            for(typename RTreeNodeType::Index ci = 0; ci < orig_node->size(); ci++) {
                ObjectID child_id = getLocCache()->iteratorID(orig_node->object(ci).object);
                if (!isInResults(child_id)) {
                    nadded++;
                    evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal, orig_node->aggregateID()) );
                    addToResults(child_id);
                }
            }
            for(typename RTreeNodeType::Index ci = 0; ci < new_node->size(); ci++) {
                ObjectID child_id = getLocCache()->iteratorID(new_node->object(ci).object);
                if (!isInResults(child_id)) {
                    nadded++;
                    evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal, new_node->aggregateID()) );
                    addToResults(child_id);
                }
            }
            // But we should end up having added just one (and at least one)
            // See note above about why this assertion isn't currently true.
            // assert(nadded == 1);
        }
        events.push_back(evt);

        // mid operation (addition causing split), no validation
    }


    void handleLiftCut(CutNodeType* cnode, RTreeNodeType* to_node) {
        validateCut();

        // This is tricky. The cutnode may be nowhere near the node we need
        // to pull up to. Instead, we have to find all cut nodes whose nodes
        // are children of to_node, destroy them, and replace them with a
        // single cut node at to_node.

        QueryEventType evt;

        // We'll exit when we have last_was_ancestor == true and
        // _is_ancestor == false, indicating we hit the end of the run for
        // this parent node.
        bool last_was_ancestor = false;
        CutNodeListIterator it;
        for(it = nodes.begin(); it != nodes.end(); ) {
            CutNodeType* node = *it;

            if ( _is_ancestor(node->rtnode, to_node) ) {
                last_was_ancestor = true;
                it = nodes.erase(it);
                length--;
                destroyCutNode(node, evt);
            }
            else {
                // If the last one was a child and we aren't then we can
                // stop traversing.  We don't advance the iterator because
                // leaving it here allows us to insert before the first
                // non-child, which should be the right place.
                if (last_was_ancestor)
                    break;
                it++;
            }
        }

        // New node insertion must happen at the end to avoid removing the
        // new node
        // NOTE: We always do this because the callback shouldn't even be
        // called unless we needed to remove one of these.
        CutNodeType* new_cnode = new CutNodeType(parent, getNativeThis(), to_node, getAggregateListener());
        if (usesAggregates()) {
            evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode, QueryEventType::Imposter) );
            addToResults(new_cnode->rtnode->aggregateID());
        }
        nodes.insert(it, new_cnode);
        length++;

        if (usesAggregates())
            events.push_back(evt);

        validateCut();
    }

    void handleReorderCut(const CutRangeVector& ranges) {
        // This kind of sucks and could probably be improved by passing more
        // information into this method about the beginning and end positions of
        // the rearranged part of the cut before rearranging the parts of the
        // cut. For now, we have to manually run through the cut and list of
        // nodes to find the beginning and end of the subsection we're going to
        // modify.
        typename CutNodeList::iterator rearrange_begin = nodes.begin();
        while(rearrange_begin != nodes.end()) {
            bool hit = false;
            for(typename CutRangeVector::const_iterator range_it = ranges.begin(); range_it != ranges.end(); range_it++) {
                if (range_it->first == *rearrange_begin) {
                    hit = true;
                    break;
                }
            }
            if (!hit)
                rearrange_begin++;
            else
                break;
        }
        assert(rearrange_begin != nodes.end());

        // Now splice together the new list
        CutNodeList new_nodes;
        // The nodes before those being rearranged
        new_nodes.splice(new_nodes.end(), nodes, nodes.begin(), rearrange_begin);
        // The rearranged nodes
        for(typename CutRangeVector::const_iterator range_it = ranges.begin(); range_it != ranges.end(); range_it++) {
            typename CutNodeList::iterator beg_it = std::find(nodes.begin(), nodes.end(), range_it->first);
            assert(beg_it != nodes.end());
            typename CutNodeList::iterator end_it = std::find(nodes.begin(), nodes.end(), range_it->second);
            assert(end_it != nodes.end());
            end_it++; // Doesn't include last item, but the end node in the
                      // range is the last node that should be included.
            new_nodes.splice(new_nodes.end(), nodes, beg_it, end_it);
        }
        // And the ones after the set being rearranged (all remaining ones)
        new_nodes.splice(new_nodes.end(), nodes, nodes.begin(), nodes.end());

        // And swap it into place. The old list should be empty at this point
        // since we've spliced everything out of it.
        nodes.swap(new_nodes);
    }

    void handleObjectInserted(CutNodeType* cnode, const LocCacheIterator& objit, int objidx) {
        // Ignore insertions/deletions during rebuild
        if (isRebuilding()) return;

        RTreeNodeType* node = cnode->rtnode;
        assert(node->leaf());

        if (usesAggregates()) {
            // When dealing with aggregates, since this node is on the cut
            // and a leaf, there are two possibilities -- the node is truly
            // in the results and none of its children are or the node is
            // not in the results and all of its children are.
            //
            // Therefore, we only have 2 choices.  If the node is in the
            // result set, we need to check the child and possibly push the
            // cut down.  If the node is not, we must simply add the child
            // as a new result.
            bool parent_in_results = isInResults(node->aggregateID());

            if (!parent_in_results) {
                // Just add the child
                ObjectID child_id = getLocCache()->iteratorID(objit);
                assert(!isInResults(child_id));

                addToResults(child_id);

                QueryEventType evt;
                evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal, node->aggregateID()) );
                events.push_back(evt);
            }
            else {
                // Check this child to decide whether to replace parent with
                // children
                bool child_satisfies = checkSatisfiesQuery(node, objit, objidx);
                if (child_satisfies) {
                    replaceParentWithChildrenResults(cnode);
                }
            }
        }
        else {
            // If we're not dealing with aggregates, we just need to check
            // if we should be adding this to the result set immediately.
            bool child_satisfies = checkSatisfiesQuery(node, objit, objidx);
            ObjectID child_id = getLocCache()->iteratorID(objit);
            updateMembership(child_id, child_satisfies);
        }
    }

    void handleObjectRemoved(CutNodeType* cnode, const LocCacheIterator& objit, bool permanent) {
        // Ignore insertions/deletions during rebuild
        if (isRebuilding()) return;

        // We just need to remove the object from the result set if we have
        // it.
        ObjectID child_id = getLocCache()->iteratorID(objit);
        removeObjectChildFromResults(child_id, permanent);

        validateCut();
    }

    // Fills in an event that corresponds to destroying the entire cut.
    void destroyCut(QueryEventType& destroyEvent) {
        // Run through the cut, adding removals to the result event
        for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); it++) {
            CutNodeType* cnode = *it;
            RTreeNodeType* node = cnode->rtnode;

            // Try to remove the node itself
            size_t node_removed = removeFromResults(node->aggregateID());
            if (node_removed > 0)
                destroyEvent.removals().push_back( typename QueryEventType::Removal(node->aggregateID(), QueryEventType::Transient) );

            // And, if its a leaf, try to remove its children
            if (node->leaf() && node_removed == 0) {
                for(int leaf_idx = 0; leaf_idx < node->size(); leaf_idx++) {
                    ObjectID leaf_id = getLocCache()->iteratorID(node->object(leaf_idx).object);
                    size_t leaf_removed = removeFromResults(leaf_id);
                    if (leaf_removed > 0)
                        destroyEvent.removals().push_back( typename QueryEventType::Removal(leaf_id, QueryEventType::Transient) );
                }
            }

            // And remove the cut node
            cnode->destroy(parent, getAggregateListener());
        }
        nodes.clear();
        length = 0;
    }

    // In order to swap trees, we need to get cuts out of the way and
    // replace them with the root of the new tree.  In order to not have
    // both trees in memory at the same time, we split this into two
    // phases. The first removes the cut from the original tree, the second
    // finishes the process by adding it to the new tree and adding the
    // updates to the result set.

    void startSwapTrees() {
        destroyCut(swapEvent);
    }

    void finishSwapTrees(RTreeNodeType* new_root) {
        // Add in the root CutNode
        if (usesAggregates()) {
            swapEvent.additions().push_back( typename QueryEventType::Addition(new_root, QueryEventType::Imposter) );
            addToResults(new_root->aggregateID());
        }
        nodes.push_back(new CutNodeType(parent, getNativeThis(), new_root, getAggregateListener()));
        length = 1;
        validateCut();

        // And finally, we can push the event onto the queue.
        if (!swapEvent.empty()) {
            events.push_back(swapEvent);
            swapEvent = QueryEventType();
        }
    }

    /** Rebuilds and actually replaces the cut node list.  Assumes that no
     *  CutNodes have become invalidated, only that they've become jumbled.
     *  This can be used when the tree is reorganized such that nodes shift
     *  position, but the overall topology of the tree does not change,
     *  e.g. if nodes are reordered for better coherence, but all the nodes
     *  should still be valid.
     */
    void rebuildCutOrder() {
        CutNodeList in_order;
        rebuildOrderedCutWithViolations(in_order, getRootRTreeNode());
        nodes.swap(in_order);
        length = nodes.size();
        //validateCut();
    }

    void validateCut() {
#ifdef PROXDEBUG
        assert(length == nodes.size());
        validateCutNodesInRTreeNodes();
        validateCutNodesInTree();
        validateCutCrossesEntireTree();
        // Now covered by validateCutOrdere
        //validateCutNodesUnrelated();
        validateCutOrdered();
        validateResultsMatchCut();
#endif //PROXDEBUG
    };

protected:

    QueryHandlerType* parent;
    QueryType* query;
    // A cut is made up of a list of CutNodes
    typedef std::list<CutNodeType*> CutNodeList;
    typedef typename CutNodeList::iterator CutNodeListIterator;
    typedef typename CutNodeList::const_iterator CutNodeListConstIterator;
    CutNodeList nodes;
    int32 length;

    typedef std::deque<QueryEventType> EventQueue;
    EventQueue events;

    typedef std::tr1::unordered_set<ObjectID, ObjectIDHasher> ResultSet;

    // Tracks modifications during a swap between trees
    QueryEventType swapEvent;

    CutBase(QueryHandlerType* _parent, QueryType* _query)
     : parent(_parent),
       query(_query),
       nodes(),
       length(0),
       events()
    {
    }
    // Always call after creation. Required because we need the
    // parent class to initialize before we can do operations (like
    // addToResults, which may depend on the parent's data).
    void init(RTreeNodeType* root) {
        if (usesAggregates()) {
            QueryEventType evt;
            evt.additions().push_back( typename QueryEventType::Addition(root, QueryEventType::Imposter) );
            addToResults(root->aggregateID());
            events.push_back(evt);
        }
        nodes.push_back(new CutNodeType(parent, getNativeThis(), root, getAggregateListener()));

        length = 1;
        validateCut();
    }
    ~CutBase() {
        for(CutNodeListIterator it = nodes.begin(); it != nodes.end(); it++) {
            CutNodeType* node = *it;
            node->destroy(parent, getAggregateListener());
        }
        nodes.clear();
        length = 0;
    }

    static RTreeNodeType* _get_root(RTreeNodeType* node) {
        RTreeNodeType* n = node;
        while(n->parent() != NULL) {
            n = n->parent();
        }
        return n;
    }

    static bool _is_ancestor(RTreeNodeType* node, RTreeNodeType* anc) {
        if (node == anc) return true;

        RTreeNodeType* n = node;
        while(n->parent() != NULL) {
            RTreeNodeType* parent = n->parent();
            if (parent == anc) return true;
            n = parent;
        }
        return false;
    }

    // Update membership in the result set based on whether it satisfies the
    // query. This version should only be used for non-aggregates.
    void updateMembership(const ObjectID& child_id, bool child_satisfies) {
        assert(!usesAggregates());
        bool in_results = isInResults(child_id);
        if (child_satisfies && !in_results) {
            addToResults(child_id);

            QueryEventType evt;
            // Note null parent is fine here because we're not using aggregates
            evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal, ObjectIDNull()()) );
            events.push_back(evt);
        }
        else if (!child_satisfies && in_results) {
            removeFromResults(child_id);

            QueryEventType evt;
            evt.removals().push_back( typename QueryEventType::Removal(child_id, QueryEventType::Transient) );
            events.push_back(evt);
        }
    }


    void removeObjectChildFromResults(const ObjectID& child_id, bool permanent) {
        bool in_results = isInResults(child_id);
        if (in_results) {
            removeFromResults(child_id);

            QueryEventType evt;
            evt.removals().push_back(
                typename QueryEventType::Removal(
                    child_id,
                    permanent ? QueryEventType::Permanent : QueryEventType::Transient
                )
            );
            events.push_back(evt);
        }
    }

    void removeObjectChildrenFromResults(RTreeNodeType* from_node) {
        // Notify any cuts that objects held by this node are gone
        assert(from_node->leaf());
        for(typename RTreeNodeType::Index idx = 0; idx < from_node->size(); idx++) {
            removeObjectChildFromResults( getLocCache()->iteratorID(from_node->object(idx).object), false );
        }
    }

    // Utility that removes and destroys a cut node, and removes results it
    // had triggered from the result set.
    void destroyCutNode(CutNodeType* node, QueryEventType& evt) {
        if (usesAggregates()) {
            // When dealing with aggregates, we first check if the
            // node itself is in the result set since if it is, none
            // of its children can be (if it is a leaf).
            // set.
            size_t nremoved = removeFromResults(node->rtnode->aggregateID());
            if (nremoved > 0) {
                evt.removals().push_back( typename QueryEventType::Removal(node->rtnode->aggregateID(), QueryEventType::Transient) );
            }
            else {
                // If it wasn't there and this is a leaf, we need to
                // check for children in the result set.  In this
                // case, they should all be there.
                removeObjectChildrenFromResults(node->rtnode);
            }
        }
        else {
            // Without aggregates, we only need to check to remove
            // children from the result set if we're at a leaf.  In
            // this case, some may be there, some may not.
            if (node->rtnode->leaf())
                removeObjectChildrenFromResults(node->rtnode);
        }
        node->destroy(parent, getAggregateListener());
    }

    CutNodeListIterator replaceParentWithChildren(const CutNodeListIterator& parent_it, QueryEventType* qevt_out) {
        CutNodeType* parent_cn = *parent_it;
        assert(!parent_cn->leaf());
        // Inserts before, so get next it
        CutNodeListIterator next_it = parent_it;
        next_it++;
        // Insert all new nodes. Going backwards leaves next_it as first of
        // new elements
        for(int i = parent_cn->rtnode->size()-1; i >=0; i--) {
            RTreeNodeType* child_rtnode = parent_cn->rtnode->node(i);
            if (qevt_out) {
                qevt_out->additions().push_back( typename QueryEventType::Addition(child_rtnode, QueryEventType::Imposter) );
                addToResults(child_rtnode->aggregateID());
            }
            next_it = nodes.insert(next_it, new CutNodeType(parent, getNativeThis(), child_rtnode, getAggregateListener()));
        }
        // Delete old node
        if (qevt_out) {
            qevt_out->removals().push_back( typename QueryEventType::Removal(parent_cn->rtnode->aggregateID(), QueryEventType::Transient) );
            removeFromResults(parent_cn->rtnode->aggregateID());
        }
        nodes.erase(parent_it);
        length += (parent_cn->rtnode->size()-1);
        // And clean up
        parent_cn->destroy(parent, getAggregateListener());

        return next_it;
    }

    // Replace the children of a leaf node (i.e. objects) with the
    // node itself.  Just adjusts the result set since
    void replaceLeafChildrenWithParent(CutNodeType* cnode, QueryEventType* qevt_out) {
        RTreeNodeType* node = cnode->rtnode;
        assert(node->leaf());
        // At leaves, if the aggregate wasn't in the results (either
        // because it had been refined or because we're not returning
        // aggregates), we need to check for children in the result set.

        // FIXME for sanity checking we could track # of removed
        // children when mWithAggregates is true and validate that
        // it is the same as the total number of children
        for(int leafidx = 0; leafidx < node->size(); leafidx++) {
            ObjectID leaf_id = getLocCache()->iteratorID(node->object(leafidx).object);
            size_t n_leaf_removed = removeFromResults(leaf_id);
            if (n_leaf_removed > 0)
                qevt_out->removals().push_back( typename QueryEventType::Removal(leaf_id, QueryEventType::Transient) );
        }

        qevt_out->additions().push_back( typename QueryEventType::Addition(node, QueryEventType::Imposter) );
        addToResults(node->aggregateID());
        // Note: no modification of length because we haven't
        // actually added or removed anything, only adjusted the
        // result set.
    }

    // Replaces children with parent in a cut.  Returns an iterator to the
    // new parent node.
    CutNodeListIterator replaceChildrenWithParent(const CutNodeListIterator& last_child_it, QueryEventType* qevt_out) {
        CutNodeListIterator child_it = last_child_it;
        RTreeNodeType* parent_rtnode = (*child_it)->rtnode->parent();
        int nchildren = parent_rtnode->size();

        // Add the new node using the parent.
        if (usesAggregates()) {
            qevt_out->additions().push_back( typename QueryEventType::Addition(parent_rtnode, QueryEventType::Imposter) );
            addToResults(parent_rtnode->aggregateID());
        }
        // Parent needs to be inserted after children, insert puts it before
        // the iterator passed in.
        CutNodeListIterator parent_insert_it = child_it;
        parent_insert_it++;
        CutNodeListIterator parent_it = nodes.insert(parent_insert_it, new CutNodeType(parent, getNativeThis(), parent_rtnode, getAggregateListener()));

        // Work backwards removing all the children.
        for(int i = nchildren-1; i >=0; i--) {
            CutNodeType* child_cn = (*child_it);
            RTreeNodeType* child_rtnode = child_cn->rtnode;
            assert(child_rtnode->parent() == parent_rtnode);
            assert(parent_rtnode->node(i) == child_rtnode);

            bool aggregate_was_in_results = false;
            // Only try to remove the child node from results for aggregates
            if (usesAggregates()) {
                size_t nremoved = removeFromResults(child_rtnode->aggregateID());
                if (nremoved > 0) {
                    aggregate_was_in_results = true;
                    qevt_out->removals().push_back( typename QueryEventType::Removal(child_rtnode->aggregateID(), QueryEventType::Transient) );
                }
            }
            // At leaves, if the aggregate wasn't in the results (either
            // because it had been refined or because we're not returning
            // aggregates), we need to check for children in the
            // result set.
            // This is almost like replaceLeafChildrenWithParent
            // but doesn't add the parent since we're in the
            // process of removing it.
            if (!aggregate_was_in_results && child_rtnode->leaf()) {
                // FIXME for sanity checking we could track # of removed
                // children when mWithAggregates is true and validate that
                // it is the same as the total number of children
                for(int leafidx = 0; leafidx < child_rtnode->size(); leafidx++) {
                    ObjectID leaf_id = getLocCache()->iteratorID(child_rtnode->object(leafidx).object);
                    size_t n_leaf_removed = removeFromResults(leaf_id);
                    if (n_leaf_removed > 0)
                        qevt_out->removals().push_back( typename QueryEventType::Removal(leaf_id, QueryEventType::Transient) );
                }
            }

            // Erase and clean up the child. Returns *next* element, so move
            // backwards to get previous child.
            child_it = nodes.erase(child_it);
            child_cn->destroy(parent, getAggregateListener());
            // We should only be able to hit nodes.begin() if we've removed
            // the last child *and* these children were the start of the cut
            assert(child_it != nodes.begin() || i == 0);
            // i > 0 is just a faster check for most iterations
            if (i > 0 || child_it != nodes.begin())
                child_it--;
        }

        length -= (parent_rtnode->size()-1);

        return parent_it;
    }

    // Replaces a parent node with children objects in the result set.  This
    // only makes sense for aggregates.  It should be used when one of the
    // child objects satisfies the constraints and therefore pulls all the
    // children in with it. Should not be used for non-leaf nodes.
    void replaceParentWithChildrenResults(CutNodeType* cnode) {
        QueryEventType evt;
        for(int i = 0; i < cnode->rtnode->size(); i++) {
            ObjectID child_id = getLocCache()->iteratorID(cnode->rtnode->object(i).object);
            addToResults(child_id);
            evt.additions().push_back( typename QueryEventType::Addition(child_id, QueryEventType::Normal, cnode->rtnode->aggregateID()) );
        }
        // For some reason this:
        //results.erase(result_it);
        // is breaking, even though I can't see how
        //result_it could ever be invalid. Instead, do
        //it the hard way and assert:
        size_t nremoved = removeFromResults(cnode->rtnode->aggregateID());
        assert(nremoved == 1);
        evt.removals().push_back( typename QueryEventType::Removal(cnode->rtnode->aggregateID(), QueryEventType::Transient) );
        events.push_back(evt);
    }


    void validateCutNodesInRTreeNodes() const {
        for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
            CutNodeType* node = *it;
            RTreeNodeType* rtnode = node->rtnode;
            assert(rtnode->findCutNode(node) != rtnode->cutNodesEnd());
        }
    }

    void validateCutNodesInTree() const {
        // Get the root base on the first cut node.  Even if this one is
        // broken, we'll be able to tell that the trees have become disjoint
        CutNodeType* first_cut_node = *(nodes.begin());
        RTreeNodeType* root = _get_root(first_cut_node->rtnode);

        for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
            CutNodeType* node = *it;
            assert(_is_ancestor(node->rtnode, root));
        }
    };

    // Validates that this cut actually crosses the entire RTree, i.e. that we
    // don't end up with wholes. This doesn't check for overlaps or anything
    // else, just that we actually have cut nodes across the tree.
    void validateCutCrossesEntireTree(RTreeNodeType* root = NULL) {
        // To avoid extra helper methods, we just set to the real root if we call
        // without a root
        if (root == NULL)
            root = parent->mRTree->root();

        // We just need to traverse recursively, stopping when we hit cut
        // nodes. If we ever hit a leaf without having hit a cut node, we have a
        // leaky cut.

        // If we hit a cut node, then we can stop. We don't index by anything
        // useful here from here, so we'll manually run through the cut nodes in
        // this node and search for a cut node with this cut as a parent.
        typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(getNativeThis());
        if (node_its != root->cutNodesEnd())
            return;

        // Otherwise, if we hit a leaf, then we've hit a leaky cut
        assert(!root->leaf());

        // And if we're not at a leaf, then we just need to recurse
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
            validateCutCrossesEntireTree(root->node(i));
    }

    // Validates that cut nodes are not through RTree nodes that are
    // ancestors of each other.
    void validateCutNodesUnrelated() const {
        for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
            CutNodeType* node = *it;
            for(CutNodeListConstIterator other_it = nodes.begin(); other_it != nodes.end(); other_it++) {
                CutNodeType* other_node = *other_it;
                if (node == other_node) continue;
                assert( ! _is_ancestor(node->rtnode, other_node->rtnode) );
                assert( ! _is_ancestor(other_node->rtnode, node->rtnode) );
            }
        }
    };

    // Validates that the nodes in a cut are in order as they cut across the
    // nodes of the RTree. This is a necessary condition for the cuts to get
    // pushed up properly.
    void validateCutOrdered() {
        // There's almost certainly a more efficient way to do this, but the
        // easiest way is to build a new list by exploring the tree in-order
        // for nodes
        CutNodeList nodes_inorder;
        rebuildOrderedCut(nodes_inorder, parent->mRTree->root());
        assert(nodes_inorder.size() == nodes.size());
        for(CutNodeListConstIterator it = nodes.begin(), other_it = nodes_inorder.begin();
            it != nodes.end(); it++, other_it++) {
            CutNodeType* node = *it;
            CutNodeType* othernode = *other_it;
            assert(node == othernode);
        }
    };

    // Rebuild an ordered cut. Works recursively.
    //
    // Note that in PROXDEBUG mode this also verifies, if a cut node was
    // found in an rtree node, that processing children doesn't change the
    // size of the cut (i.e. we didn't find a cutnode in a subtree where we
    // shouldn't).  This is more expensive, but also covers the
    // functionality of validateCutNodesUnrelated.
    void rebuildOrderedCut(CutNodeList& inorder, RTreeNodeType* root) {
        bool had_cut = false;
        typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(getNativeThis());
        if (node_its != root->cutNodesEnd()) {
            CutNodeType* cnode = node_its->second;
            assert(cnode->parent == this);
            inorder.push_back(cnode);
            had_cut = true;
        }

        if (root->leaf())
            return;

        // With PROXDEBUG we verify no children get added if we process
        // children nodes, i.e. that we don't have sibling cutnodes in
        // ancestor rtree nodes.  Without PROXDEBUG, only processes children
        // nodes if a cut node wasn't found at this node, efficiently
        // culling the tree.
#if !defined(PROXDEBUG)
        if (!had_cut) {
#else
            int num_before_children = inorder.size();
#endif

            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                rebuildOrderedCut(inorder, root->node(i));
#if defined(PROXDEBUG)
            int num_after_children = inorder.size();
            assert(!had_cut || num_before_children == num_after_children);
#else
        }
#endif
    };


    // Helper for rebuildOrderedCutWithViolations. Looks for CutNodes in a
    // subtree and a) removes them and b) removes any results they had from
    // the result set.
    void rebuildOrderedCutWithViolations_removeChildrenCutNodes(QueryEventType& evt, RTreeNodeType* root) {
        // First, check for a cut node at this bvh node
        typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(getNativeThis());
        if (node_its != root->cutNodesEnd()) {
            CutNodeType* cnode = node_its->second;
            assert(cnode->parent == this);
            destroyCutNode(cnode, evt);
        }

        // Then, recurse and check within children
        if (root->leaf()) return;
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
            rebuildOrderedCutWithViolations_removeChildrenCutNodes(evt, root->node(i));
    };

    // Driver for rebuildOrderedCutWithViolations first pass. Scans the tree
    // with a pre-order traversal to filter out cut nodes that appear
    // beneath other cut nodes.
    void rebuildOrderedCutWithViolations_filterChildrenPass(QueryEventType& evt, RTreeNodeType* root) {
        typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(getNativeThis());

        if (root->leaf()) return;

        // If there's a cut node here, remove all children cut nodes
        if (node_its != root->cutNodesEnd()) {
            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                rebuildOrderedCutWithViolations_removeChildrenCutNodes(evt, root->node(i));
        } // Otherwise, recurse
        else {
            for(typename RTreeNodeType::Index i = 0; i < root->size(); i++)
                rebuildOrderedCutWithViolations_filterChildrenPass(evt, root->node(i));
        }
    };

    // Driver for rebuildOrderedCutWithViolations second pass. Scans through
    // looking for gaps and inserts new CutNodes.
    bool rebuildOrderedCutWithViolations_fillGapsPass(QueryEventType& evt, RTreeNodeType* root, bool treat_as_root = false) {
        // The basic approach is to process all children of this node
        // recursively and record whether the child had data filled in
        // (returned true).  If no children found CutNodes, then no data
        // would be filled in and we just return, leaving the cut to be
        // handled at a higher level.  If any children did have nodes, then
        // those that returned that they were empty get a cut node and those
        // that returned that they found a cut node should already be
        // filled.

        // If there's a cut node of ours on this node, we're done with this subtree
        typename RTreeNodeType::CutNodeListConstIterator node_its = root->findCutNode(getNativeThis());
        if (node_its != root->cutNodesEnd()) return true;

        // Base case: at a leaf, there's no additional processing to be
        // done. This subtree is empty.
        if (root->leaf()) return false;

        // Next, process each of the children, recording whether they are
        // filled or not.
        std::vector<bool> children_results;
        bool any_child_was_filled = false;
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++) {
            bool child_was_filled = rebuildOrderedCutWithViolations_fillGapsPass(evt, root->node(i));
            children_results.push_back(child_was_filled);
            any_child_was_filled = (any_child_was_filled || child_was_filled);
        }

        // Now, either everything was empty...
        if (!any_child_was_filled) {
            // If we're the root and all children were empty, fill in a
            // CutNode for us.
            if (treat_as_root) {
                CutNodeType* new_cnode = new CutNodeType(parent, getNativeThis(), root, getAggregateListener());
                if (usesAggregates()) {
                    evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode, QueryEventType::Imposter) );
                    addToResults(new_cnode->rtnode->aggregateID());
                }
            }
            return false;
        }
        // Or we need to fill in the empties
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++) {
            if (children_results[i] == true) // Already filled
                continue;
            // Add a CutNode for this child
            CutNodeType* new_cnode = new CutNodeType(parent, getNativeThis(), root->node(i), getAggregateListener());
            if (usesAggregates()) {
                evt.additions().push_back( typename QueryEventType::Addition(new_cnode->rtnode, QueryEventType::Imposter) );
                addToResults(new_cnode->rtnode->aggregateID());
            }
            // Not adding to list since we're rebuilding it in the next pass
        }
        return true;
    }

    // Rebuild an ordered cut. Works recursively. Handles cut nodes that
    // have been resorted such that there are parent/child violations and
    // full cut violations: cut nodes may have been reordered such that
    // there are gaps and there are cut nodes in subtrees of other cut
    // nodes.
    void rebuildOrderedCutWithViolations(CutNodeList& inorder, RTreeNodeType* root) {
        // This works in two passes.
        QueryEventType evt;
        // On the first pass, we make sure we don't have any overlapping cut
        // nodes, i.e. that reordering hasn't caused the rtnode of one cut
        // node to become the child of the rtnode of another cut node.
        rebuildOrderedCutWithViolations_filterChildrenPass(evt, root);
        // On the second pass, we look for gaps and fill them in with new
        // CutNodes. Last parameter indicates that, if all children were
        // empty we should treat this as the root and make sure a cut node
        // exists.
        rebuildOrderedCutWithViolations_fillGapsPass(evt, root, true);
        // On the third pass, we actually rebuild the cut.  This just uses
        // the normal approach since the previous passes guarantee gap-free,
        // non-overlapping CutNodes.
        rebuildOrderedCut(inorder, root);
        // Save the adjustments triggered by this.
        if (evt.size() > 0)
            events.push_back(evt);
    }

    // Validates that all entries in a subtree (aggregates and
    // non-aggregates) are *not* in the result set.
    void validateSubtreeObjectsNotInResults(RTreeNodeType* root) {
        assert(!isInResults(root->aggregateID()));
        validateChildrenSubtreesObjectsNotInResults(root);
    }

    void validateChildrenSubtreesObjectsNotInResults(RTreeNodeType* root) {
        for(typename RTreeNodeType::Index i = 0; i < root->size(); i++) {
            if (root->leaf())
                assert( !isInResults(getLocCache()->iteratorID(root->object(i).object)) );
            else
                validateSubtreeObjectsNotInResults(root->node(i));
        }
    }

    // Validates that the result set matches the nodes the cut goes through,
    // checking both for missing entries (e.g. one leaf object out of 5
    // children is missing) and for extra entries (e.g. a cut moved, but
    // somebody neglected to remove the entry or child objects' entries from
    // results).
    void validateResultsMatchCut() {
        ResultSet accounted;
        for(CutNodeListConstIterator it = nodes.begin(); it != nodes.end(); it++) {
            CutNodeType* node = *it;
            RTreeNodeType* rtnode = node->rtnode;
            if (usesAggregates()) {
                // Check for the aggregate and invalidate children
                if (isInResults(rtnode->aggregateID())) {
                    accounted.insert(rtnode->aggregateID());
                    validateChildrenSubtreesObjectsNotInResults(rtnode);
                }
                else { // Otherwise, we better have all the children
                    for(typename RTreeNodeType::Index i = 0; i < rtnode->size(); i++)
                        accounted.insert( rtnode->leaf() ? getLocCache()->iteratorID(rtnode->object(i).object) : rtnode->node(i)->aggregateID() );
                }
            }
            else {
                // Without aggregates, we should have some subset of the
                // children of the node.
                if (!rtnode->leaf()) continue;
                // To avoid actually evaluating, we're conservative in this
                // case and might miss some false positives. We just add all
                // leaf children we encounter
                for(typename RTreeNodeType::Index i = 0; i < rtnode->size(); i++)
                    accounted.insert( getLocCache()->iteratorID(rtnode->object(i).object) );
            }
        }

        // Now that we've collected the information, we can report errors.

        // Accounted - results = objects that are missing from the results
        // We can only do this with aggregates because only w/ aggregates are we
        // forced to have a full cut. Otherwise we'd actually have to retest
        // everything for membership in the results.
        if (usesAggregates()) {
            assert( accounted.size() == getResultsSize() );
            for(typename ResultSet::iterator it = accounted.begin(); it != accounted.end(); it++)
                assert( isInResults(*it) );
        }
    }
};


} // namespace Prox

#endif // _PROX_RTREE_CUT_HPP_
