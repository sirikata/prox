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
    assert( mObjects.find(obj->id()) == mObjects.end() );
    mObjects[obj->id()] = obj;
}

void ObjectLocationServiceCache::removeObject(const Object* obj) {
    ObjectMap::iterator it = mObjects.find(obj->id());
    assert( it != mObjects.end() );
    mObjects.erase(it);
}


LocationServiceCache::Iterator ObjectLocationServiceCache::startTracking(const ObjectID& id) {
    Object* obj = lookup(id);
    assert(obj != NULL);
    obj->addUpdateListener(this);
    assert(obj != NULL);
    return Iterator((void*)obj);
}

void ObjectLocationServiceCache::stopTracking(const Iterator& id) {
    Object* obj = (Object*)id.data;
    assert(obj != NULL);
    obj->removeUpdateListener(this);
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
    assert( mListeners.find(listener) == mListeners.end() );
    mListeners.insert(listener);
}

void ObjectLocationServiceCache::removeUpdateListener(LocationUpdateListenerType* listener) {
    ListenerSet::iterator it = mListeners.find(listener);
    assert( it != mListeners.end() );
    mListeners.erase(it);
}

Object* ObjectLocationServiceCache::lookup(const ObjectID& id) const {
    ObjectMap::const_iterator it = mObjects.find(id);
    if (it == mObjects.end()) return NULL;
    return it->second;
}

void ObjectLocationServiceCache::objectCreated(const Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationConnected(obj->id(), true, pos, BoundingSphere(bounds.center(), 0), bounds.radius());
}

void ObjectLocationServiceCache::objectPositionUpdated(Object* obj, const MotionVector3& old_pos, const MotionVector3& new_pos) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationPositionUpdated(obj->id(), old_pos, new_pos);
}

void ObjectLocationServiceCache::objectBoundsUpdated(Object* obj, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
    BoundingSphere old_region(old_bounds.center(), 0);
    BoundingSphere new_region(new_bounds.center(), 0);
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++) {
        (*it)->locationRegionUpdated(obj->id(), old_region, new_region);
        (*it)->locationMaxSizeUpdated(obj->id(), old_bounds.radius(), new_bounds.radius());
    }
}

void ObjectLocationServiceCache::objectDeleted(const Object* obj) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(obj->id());

    removeObject(obj);
}

void ObjectLocationServiceCache::simulatorAddedObject(Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationConnected(obj->id(), true, pos, BoundingSphere(bounds.center(), 0), bounds.radius());
}

void ObjectLocationServiceCache::simulatorRemovedObject(Object* obj) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(obj->id());
}

void ObjectLocationServiceCache::simulatorAddedQuery(Query* query) {
    // Only using this interface for tracking object additions
}

void ObjectLocationServiceCache::simulatorRemovedQuery(Query* query) {
    // Only using this interface for tracking object additions
}

} // namespace Simulation
} // namespace Prox
