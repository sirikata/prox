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

#include "ObjectLocationServiceCache.hpp"

namespace Prox {
namespace Simulation {

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


void ObjectLocationServiceCache::startTracking(const ObjectID& id) {
    Object* obj = lookup(id);
    assert(obj != NULL);
    obj->addUpdateListener(this);
}

void ObjectLocationServiceCache::stopTracking(const ObjectID& id) {
    Object* obj = lookup(id);
    assert(obj != NULL);
    obj->removeUpdateListener(this);
}


const MotionVector3& ObjectLocationServiceCache::location(const ObjectID& id) const {
    Object* obj = lookup(id);
    assert(obj != NULL);
    return obj->position();
}

const BoundingSphere& ObjectLocationServiceCache::bounds(const ObjectID& id) const {
    Object* obj = lookup(id);
    assert(obj != NULL);
    return obj->bounds();
}


void ObjectLocationServiceCache::addUpdateListener(LocationUpdateListener* listener) {
    assert( mListeners.find(listener) == mListeners.end() );
    mListeners.insert(listener);
}

void ObjectLocationServiceCache::removeUpdateListener(LocationUpdateListener* listener) {
    ListenerSet::iterator it = mListeners.find(listener);
    assert( it != mListeners.end() );
    mListeners.erase(it);
}

Object* ObjectLocationServiceCache::lookup(const ObjectID& id) const {
    ObjectMap::const_iterator it = mObjects.find(id);
    if (it == mObjects.end()) return NULL;
    return it->second;
}

void ObjectLocationServiceCache::objectPositionUpdated(Object* obj, const MotionVector3& old_pos, const MotionVector3& new_pos) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationPositionUpdated(obj->id(), old_pos, new_pos);
}

void ObjectLocationServiceCache::objectBoundsUpdated(Object* obj, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationBoundsUpdated(obj->id(), old_bounds, new_bounds);
}

void ObjectLocationServiceCache::objectDeleted(const Object* obj) {
    for(ListenerSet::iterator it = mListeners.begin(); it != mListeners.end(); it++)
        (*it)->locationDisconnected(obj->id());

    removeObject(obj);
}

} // namespace Simulation
} // namespace Prox
