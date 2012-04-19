// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_BASE_IMPL_NODE_ITERATOR_HPP_
#define _PROX_BASE_IMPL_NODE_ITERATOR_HPP_

namespace Prox {
namespace QueryHandlerBaseImpl {


template<typename SimulationTraits>
class NodeIterator;
template<typename SimulationTraits>
class NodeIteratorImpl;


template<typename SimulationTraits>
class NodeIterator {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::TimeType Time;
    typedef NodeIteratorImpl<SimulationTraits> NodeIteratorImplType;

    // We want iterators to have value semantics, so we can't just share a
    // reference counted pointer or something like
    //  Iterator begin = cont.begin();
    //  Iterator it = begin;
    //  it++;
    //  assert(it != begin);
    // Wouldn't work since the underlying implementation would be
    // shared. Instead, we use a copy-on-write if refcount > 1. This keeps
    // memory allocations down (e.g. from just passing iterators through methods
    // by value for reading from), but keeps the value semantics by paying for
    // the allocation when it might not be safe to share the same underlying
    // data anymore.

    explicit NodeIterator()
     : impl(NULL)
    {
    }
    NodeIterator(NodeIteratorImplType* i)
     : impl(i)
    {
        impl->_incref();
    }
    NodeIterator(const NodeIterator& rhs)
     : impl(rhs.impl)
    {
        if (impl != NULL)
            impl->_incref();
    }
    ~NodeIterator() {
        if (impl != NULL)
            impl->_decref();
    }

    NodeIterator& operator=(const NodeIterator& rhs) {
        if (impl != NULL)
            impl->_decref();
        impl = rhs.impl;
        if (impl != NULL)
            impl->_incref();
        return *this;
    }

    // Traversal
    NodeIterator& operator++() {
        _next();
        return *this;
    }
    void operator++(int) {
        // Don't return the old value since this requires making a copy of
        // the iterator impl
        _next();
    }

    // Comparison
    bool operator==(const NodeIterator& rhs) {
        if (impl == NULL)
            return rhs.impl == NULL;
        return impl->_equals(rhs.impl);
    }
    bool operator!=(const NodeIterator& rhs) {
        return !(*this == rhs);
    }

    // Get data
    const ObjectID& id() const {
        assert(impl != NULL);
        return impl->id();
    }
    ObjectID parentId() const {
        assert(impl != NULL);
        return impl->parentId();
    }
    BoundingSphere bounds(const Time& t) const {
        assert(impl != NULL);
        return impl->bounds(t);
    }
    uint32 cuts() const {
        assert(impl != NULL);
        return impl->cuts();
    }
private:
    // Move to the next item, cloning underlying iterator if necessary to avoid
    // invalidating other iterators
    void _next() {
        assert(impl != NULL);
        if (impl->_refcount() > 1) {
            NodeIteratorImplType* impl_new = impl->_clone();
            impl_new->_incref();
            impl->_decref();
            impl = impl_new;
        }
        impl->next();
    }
    NodeIteratorImplType* impl;
};


// This is the implementation interface of NodeIterators. The main
// NodeIterator class wraps this, which should be implemented by each
// implementation of this query processor interface. It's reference counted
// so it will get cleaned up automatically.
template<typename SimulationTraits>
class NodeIteratorImpl {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::TimeType Time;

    NodeIteratorImpl()
     : _ref_count(0)
    {}
    virtual ~NodeIteratorImpl() {}

    void _incref() { _ref_count++; }
    void _decref() {
        _ref_count--;
        assert(_ref_count >= 0);
        if (_ref_count == 0) {
            delete this;
        }
    }
    int32 _refcount() { return _ref_count; }

    // Cloning for when sharing iterator data becomes unsafe. Should return a
    // copy except that the refcount should be 0 on the new one, just as if it
    // were allocated from scratch.
    virtual NodeIteratorImpl* _clone() = 0;
    // Traversal
    virtual void next() = 0;
    // Comparison
    virtual bool _equals(NodeIteratorImpl* rhs) = 0;
    // Get data
    virtual const ObjectID& id() const = 0;
    virtual ObjectID parentId() const = 0;
    virtual BoundingSphere bounds(const Time& t) const = 0;
    virtual uint32 cuts() const = 0;
private:
    int32 _ref_count;
};


} // namespace QueryHandlerBaseImpl
} // namespace Prox

#endif //_PROX_BASE_IMPL_NODE_ITERATOR_HPP_
