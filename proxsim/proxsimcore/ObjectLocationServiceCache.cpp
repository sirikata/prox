/*  proxsim
 *  ObjectLocationServiceCache.cpp
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

#include <stdint.h>
#include "ObjectLocationServiceCache.hpp"

namespace Prox {
namespace Simulation {

BoundingSphere ObjectLocationServiceCache::sNullBoundingSphere = BoundingSphere(Reference::Vector3f(0,0,0), 0.0f);

ObjectLocationServiceCache::ObjectLocationServiceCache() {
}

ObjectLocationServiceCache::~ObjectLocationServiceCache() {
}

void ObjectLocationServiceCache::addObject(Object* obj) {
    Lock lck(mMutex);

    // Since these are refcounted, we may have a copy, but it *must* be marked
    // as !exists
    ObjectMap::iterator it = mObjects.find(obj->id());
    assert( it == mObjects.end() || it->second.exists == false );
    if (it == mObjects.end())
        mObjects[obj->id()] = ObjectInfo(obj);
    else
        mObjects[obj->id()].exists = true;
}

void ObjectLocationServiceCache::removeObject(const Object* obj) {
    Lock lck(mMutex);

    ObjectMap::iterator it = mObjects.find(obj->id());
    assert( it != mObjects.end() );
    it->second.exists = false;

    tryClearObject(it);
}

void ObjectLocationServiceCache::tryClearObject(ObjectMap::iterator& it) {
    if (it->second.refcount == 0 && !it->second.exists)
        mObjects.erase(it);
}

void ObjectLocationServiceCache::addPlaceholderImposter(
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
    mObjects[id] = ObjectInfo(NULL);
}

LocationServiceCache::Iterator ObjectLocationServiceCache::startTracking(const ObjectID& id) {
    Lock lck(mMutex);

    ObjectMap::iterator it = mObjects.find(id);
    assert(it != mObjects.end());

    it->second.refcount++;
    // Aggregates will have NULL object and don't need update tracking
    if (it->second.refcount == 1 && it->second.object != NULL)
        it->second.object->addUpdateListener(this);
    return Iterator((void*)it->second.object);
}

void ObjectLocationServiceCache::stopTracking(const Iterator& id) {
    Lock lck(mMutex);

    Object* obj = (Object*)id.data;
    assert(obj != NULL);

    stopRefcountTracking(obj->id());
}

bool ObjectLocationServiceCache::startRefcountTracking(const ObjectID& id) {
    Lock lck(mMutex);
    // Just reuse existing code and ignore the output iterator, which
    // can just be discarded since it's just a pointer to the existing
    // object. Not sure when it would be reasonable for this to fail,
    // so the return will always assume success...
    startTracking(id);
    return true;
}

void ObjectLocationServiceCache::stopRefcountTracking(const ObjectID& objid) {
    Lock lck(mMutex);

    ObjectMap::iterator it = mObjects.find(objid);
    assert(it != mObjects.end());
    it->second.refcount--;

    // Aggregates will have NULL object and don't need update tracking
    if (it->second.refcount == 0 && it->second.object != NULL)
        it->second.object->removeUpdateListener(this);

    tryClearObject(it);
}


MotionVector3 ObjectLocationServiceCache::location(const Iterator& id) {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    return obj->position();
}

Vector3 ObjectLocationServiceCache::centerOffset(const Iterator& id) {
    // We don't do any offsets in this test code
    return Vector3::nil();
}

float32 ObjectLocationServiceCache::centerBoundsRadius(const Iterator& id) {
    // Single objects have zero sized bounds around center points
    return 0;
}

float32 ObjectLocationServiceCache::maxSize(const Iterator& id) {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    return obj->bounds().radius();
}

ZernikeDescriptor& ObjectLocationServiceCache::zernikeDescriptor(const Iterator& id) {
    // This test code doesn't currently support meshes/zernike descriptors
    static ZernikeDescriptor dummy_zd;
    return dummy_zd;
}

String ObjectLocationServiceCache::mesh(const Iterator& id) {
    // This test code doesn't currently support meshes/zernike descriptors
    return "";
}

bool ObjectLocationServiceCache::isLocal(const Iterator& id) {
    return true; // We don't deal with replicas in the simulation
}

const ObjectID& ObjectLocationServiceCache::iteratorID(const Iterator& id) {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    return obj->id();
}

void ObjectLocationServiceCache::addUpdateListener(LocationUpdateListenerType* listener) {
    Lock lck(mMutex);

    assert( mListeners.find(listener) == mListeners.end() );
    mListeners.insert(listener);
}

void ObjectLocationServiceCache::removeUpdateListener(LocationUpdateListenerType* listener) {
    Lock lck(mMutex);

    ListenerSet::iterator it = mListeners.find(listener);
    assert( it != mListeners.end() );
    mListeners.erase(it);
}

void ObjectLocationServiceCache::objectCreated(const Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) {
    Lock lck(mMutex);

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationConnected(obj->id(), false, true, pos, BoundingSphere(bounds.center(), 0), bounds.radius());
}

void ObjectLocationServiceCache::objectPositionUpdated(Object* obj, const MotionVector3& old_pos, const MotionVector3& new_pos) {
    Lock lck(mMutex);

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationPositionUpdated(obj->id(), old_pos, new_pos);
}

void ObjectLocationServiceCache::objectBoundsUpdated(Object* obj, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
    Lock lck(mMutex);

    BoundingSphere old_region(old_bounds.center(), 0);
    BoundingSphere new_region(new_bounds.center(), 0);
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++) {
        (*it)->locationRegionUpdated(obj->id(), old_region, new_region);
        (*it)->locationMaxSizeUpdated(obj->id(), old_bounds.radius(), new_bounds.radius());
    }
}

void ObjectLocationServiceCache::objectDeleted(const Object* obj) {
    Lock lck(mMutex);

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(obj->id());

    removeObject(obj);
}

void ObjectLocationServiceCache::simulatorAddedObject(Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) {
    Lock lck(mMutex);

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationConnected(obj->id(), false, true, pos, BoundingSphere(bounds.center(), 0), bounds.radius());
}

void ObjectLocationServiceCache::simulatorRemovedObject(Object* obj) {
    Lock lck(mMutex);

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(obj->id());
}

} // namespace Simulation
} // namespace Prox
