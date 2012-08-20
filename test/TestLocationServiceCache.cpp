// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "TestLocationServiceCache.hpp"

TestLocationServiceCache::TestLocationServiceCache() {
}

TestLocationServiceCache::~TestLocationServiceCache() {
}

void TestLocationServiceCache::addObject(const ObjectID& objid,
    bool aggregate,
    const MotionVector3& loc,
    const Vector3& bounds_center_offset,
    const float32 bounds_center_bounds_radius,
    const float32 bounds_max_size,
    const String& mesh,
    bool notify)
{
    // Since these are refcounted, we may have a copy, but it *must* be marked
    // as !exists. The one exception is for aggregates, which will have been
    // added via addPlaceholderImposter already
    ObjectMap::iterator it = mObjects.find(objid);
    assert( it == mObjects.end() || it->second->exists == false || aggregate);
    if (it == mObjects.end())
        mObjects[objid] = ObjectInfoPtr(
            new ObjectInfo(objid, aggregate, loc, bounds_center_offset, bounds_center_bounds_radius, bounds_max_size, mesh)
        );
    else
        mObjects[objid]->exists = true;

    if (notify)
        for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
            (*it)->locationConnected(objid, aggregate, true, loc, BoundingSphere(bounds_center_offset, bounds_center_bounds_radius), bounds_max_size);
}

void TestLocationServiceCache::addObjectWithParent(
    const ObjectID& objid,
    const ObjectID& parentid,
    bool aggregate,
    const MotionVector3& loc,
    const Vector3& bounds_center_offset,
    const float32 bounds_center_bounds_radius,
    const float32 bounds_max_size,
    const String& mesh,
    bool notify)
{
    // Since these are refcounted, we may have a copy, but it *must* be marked
    // as !exists. The one exception is for aggregates, which will have been
    // added via addPlaceholderImposter already
    ObjectMap::iterator it = mObjects.find(objid);
    assert( it == mObjects.end() || it->second->exists == false || aggregate);
    if (it == mObjects.end())
        mObjects[objid] = ObjectInfoPtr(
            new ObjectInfo(objid, aggregate, loc, bounds_center_offset, bounds_center_bounds_radius, bounds_max_size, mesh)
        );
    else
        mObjects[objid]->exists = true;

    if (notify)
        for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
            (*it)->locationConnectedWithParent(objid, parentid, aggregate, true, loc, BoundingSphere(bounds_center_offset, bounds_center_bounds_radius), bounds_max_size);
}

#define LOCK_AND_GET_OBJ_ENTRY(it, objid)               \
    ObjectMap::iterator it = mObjects.find(objid);      \
    assert( it != mObjects.end() )

void TestLocationServiceCache::removeObject(const ObjectID& objid) {
    LOCK_AND_GET_OBJ_ENTRY(it, objid);
    it->second->exists = false;
    // Make sure this stays alive until we're done
    it->second->refcount++;

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(objid);

    // Then allow it to possibly be cleaned up
    it->second->refcount--;
    tryClearObject(it);
}

void TestLocationServiceCache::updateLocation(const ObjectID& objid, const MotionVector3& newval) {
    LOCK_AND_GET_OBJ_ENTRY(it, objid);

    MotionVector3 oldval = it->second->loc;
    it->second->loc = newval;

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationPositionUpdated(objid, oldval, newval);
}

void TestLocationServiceCache::updateBounds(const ObjectID& objid,
    const Vector3& bounds_center_offset,
    const float32 bounds_center_bounds_radius,
    const float32 bounds_max_size)
{
    LOCK_AND_GET_OBJ_ENTRY(it, objid);

    Vector3 old_bounds_center_offset = it->second->bounds_center_offset;
    float32 old_bounds_center_bounds_radius = it->second->bounds_center_bounds_radius;
    float32 old_bounds_max_size = it->second->bounds_max_size;
    it->second->bounds_center_offset = bounds_center_offset;
    it->second->bounds_center_bounds_radius = bounds_center_bounds_radius;
    it->second->bounds_max_size = bounds_max_size;

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++) {
        (*it)->locationRegionUpdated(objid, BoundingSphere(old_bounds_center_offset, old_bounds_center_bounds_radius), BoundingSphere(bounds_center_offset, bounds_center_bounds_radius));
        (*it)->locationMaxSizeUpdated(objid, old_bounds_max_size, bounds_max_size);
    }
}

void TestLocationServiceCache::updateMesh(const ObjectID& objid, const String& newval) {
    LOCK_AND_GET_OBJ_ENTRY(it, objid);
    it->second->mesh = newval;
}

void TestLocationServiceCache::tryClearObject(ObjectMap::iterator& it) {
    if (it->second->refcount == 0 && !it->second->exists)
        mObjects.erase(it);
}

void TestLocationServiceCache::addPlaceholderImposter(
    const ObjectID& id,
    const Vector3& center_offset,
    const float32 center_bounds_radius,
    const float32 max_size,
    const String& zernike,
    const String& mesh
) {
    // We don't actually have a corresponding object for this, and we don't have
    // a way to use this data for it by creating one. Instead, we'll fill in a
    // bogus entry which shouldn't be accessed.
    mObjects[id] = ObjectInfoPtr(
        new ObjectInfo(
            id,
            true,
            MotionVector3(Time::null(), Vector3(center_offset), Vector3::nil()),
            Vector3::nil(),
            center_bounds_radius,
            max_size,
            mesh
        )
    );
}

TestLocationServiceCache::Iterator TestLocationServiceCache::startTracking(const ObjectID& id) {
    ObjectMap::iterator it = mObjects.find(id);
    assert(it != mObjects.end());

    it->second->refcount++;
    return Iterator((void*)new ObjectInfoPtr(it->second));
}

void TestLocationServiceCache::stopTracking(const Iterator& id) {
    ObjectInfoPtr* obj = (ObjectInfoPtr*)id.data;
    assert(obj != NULL || !*obj);

    stopRefcountTracking((*obj)->objid);
    delete obj;
}

bool TestLocationServiceCache::startRefcountTracking(const ObjectID& id) {
    // Just reuse existing code and ignore the output iterator, which
    // can just be discarded since it's just a pointer to the existing
    // object. Not sure when it would be reasonable for this to fail,
    // so the return will always assume success...
    startTracking(id);
    return true;
}

void TestLocationServiceCache::stopRefcountTracking(const ObjectID& objid) {
    ObjectMap::iterator it = mObjects.find(objid);
    assert(it != mObjects.end());
    it->second->refcount--;

    tryClearObject(it);
}

#define GET_OBJECT_INFO_PTR(name, from)                     \
    ObjectInfoPtr* name = (ObjectInfoPtr*)from.data;       \
    assert(obj != NULL || !*obj);

TestLocationServiceCache::MotionVector3 TestLocationServiceCache::location(const Iterator& id) {
    GET_OBJECT_INFO_PTR(obj, id);
    return (*obj)->loc;
}

TestLocationServiceCache::Vector3 TestLocationServiceCache::centerOffset(const Iterator& id) {
    GET_OBJECT_INFO_PTR(obj, id);
    return (*obj)->bounds_center_offset;
}

TestLocationServiceCache::float32 TestLocationServiceCache::centerBoundsRadius(const Iterator& id) {
    GET_OBJECT_INFO_PTR(obj, id);
    return (*obj)->bounds_center_bounds_radius;
}

TestLocationServiceCache::float32 TestLocationServiceCache::maxSize(const Iterator& id) {
    GET_OBJECT_INFO_PTR(obj, id);
    return (*obj)->bounds_max_size;
}

TestLocationServiceCache::ZernikeDescriptor& TestLocationServiceCache::zernikeDescriptor(const Iterator& id) {
    // This test code doesn't currently support meshes/zernike descriptors
    static ZernikeDescriptor dummy_zd;
    return dummy_zd;
}

TestLocationServiceCache::String TestLocationServiceCache::mesh(const Iterator& id) {
    GET_OBJECT_INFO_PTR(obj, id);
    return (*obj)->mesh;
}

bool TestLocationServiceCache::isLocal(const Iterator& id) {
    return true; // We don't deal with replicas in the simulation
}

const TestLocationServiceCache::ObjectID& TestLocationServiceCache::iteratorID(const Iterator& id) {
    GET_OBJECT_INFO_PTR(obj, id);
    return (*obj)->objid;
}

void TestLocationServiceCache::addUpdateListener(LocationUpdateListenerType* listener) {
    assert( mListeners.find(listener) == mListeners.end() );
    mListeners.insert(listener);
}

void TestLocationServiceCache::removeUpdateListener(LocationUpdateListenerType* listener) {
    ListenerSet::iterator it = mListeners.find(listener);
    assert( it != mListeners.end() );
    mListeners.erase(it);
}
