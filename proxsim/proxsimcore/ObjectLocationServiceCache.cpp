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

    assert( mObjects.find(obj->id()) == mObjects.end() );
    mObjects[obj->id()] = ObjectInfo(obj);
}

void ObjectLocationServiceCache::removeObject(const Object* obj) {
    Lock lck(mMutex);

    ObjectMap::iterator it = mObjects.find(obj->id());
    assert( it != mObjects.end() );
    it->second.exists = false;

    tryClearObject(obj);
}

void ObjectLocationServiceCache::tryClearObject(const Object* obj) {
    ObjectMap::iterator it = mObjects.find(obj->id());
    if (it->second.refcount == 0 && !it->second.exists)
        mObjects.erase(it);
}

LocationServiceCache::Iterator ObjectLocationServiceCache::startTracking(const ObjectID& id) {
    Lock lck(mMutex);

    ObjectMap::iterator it = mObjects.find(id);
    assert(it != mObjects.end());

    it->second.refcount++;
    if (it->second.refcount == 1)
        it->second.object->addUpdateListener(this);
    return Iterator((void*)it->second.object);
}

void ObjectLocationServiceCache::stopTracking(const Iterator& id) {
    Lock lck(mMutex);

    Object* obj = (Object*)id.data;
    assert(obj != NULL);

    ObjectMap::iterator it = mObjects.find(obj->id());
    assert(it != mObjects.end());
    it->second.refcount--;

    if (it->second.refcount == 0)
        obj->removeUpdateListener(this);

    tryClearObject(obj);
}


const MotionVector3& ObjectLocationServiceCache::location(const Iterator& id) const {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    return obj->position();
}

const BoundingSphere& ObjectLocationServiceCache::region(const Iterator& id) const {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    assert(obj->bounds().center() == Reference::Vector3f::nil());
    return sNullBoundingSphere;
}

float32 ObjectLocationServiceCache::maxSize(const Iterator& id) const {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    return obj->bounds().radius();
}

bool ObjectLocationServiceCache::isLocal(const Iterator& id) const {
    return true; // We don't deal with replicas in the simulation
}

const ObjectID& ObjectLocationServiceCache::iteratorID(const Iterator& id) const {
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
        (*it)->locationConnected(obj->id(), true, pos, BoundingSphere(bounds.center(), 0), bounds.radius());
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
        (*it)->locationConnected(obj->id(), true, pos, BoundingSphere(bounds.center(), 0), bounds.radius());
}

void ObjectLocationServiceCache::simulatorRemovedObject(Object* obj) {
    Lock lck(mMutex);

    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(obj->id());
}

} // namespace Simulation
} // namespace Prox
