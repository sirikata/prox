/*  proxsim
 *  Object.cpp
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

#include "Object.hpp"
#include <algorithm>

namespace Prox {
namespace Simulation {

Object::Object(const ObjectID& id, const MotionVector3& c, const BoundingSphere& bs)
 : mID(id),
   mPosition(c),
   mBounds(bs)
{
}

Object::Object(const Object& cpy)
 : mID(cpy.mID),
   mPosition(cpy.mPosition),
   mBounds(cpy.mBounds)
{
}

Object::~Object() {
    for(UpdateListenerList::iterator it = mUpdateListeners.begin(); it != mUpdateListeners.end(); it++)
        (*it)->objectDeleted(this);
}

const ObjectID& Object::id() const {
    return mID;
}

const MotionVector3& Object::position() const {
    return mPosition;
}

Vector3 Object::position(const Time& t) const {
    return mPosition.position(t);
}

const BoundingSphere& Object::bounds() const {
    return mBounds;
}

BoundingSphere Object::worldBounds(const Time& t) const {
    return BoundingSphere( mBounds.center() + mPosition.position(t), mBounds.radius() );
}

void Object::position(const MotionVector3& new_pos) {
    MotionVector3 old_pos = mPosition;
    mPosition = new_pos;
    for(UpdateListenerList::iterator it = mUpdateListeners.begin(); it != mUpdateListeners.end(); it++)
        (*it)->objectPositionUpdated(this, old_pos, new_pos);
}

void Object::bounds(const BoundingSphere& new_bounds) {
    BoundingSphere old_bounds = mBounds;
    mBounds = new_bounds;
    for(UpdateListenerList::iterator it = mUpdateListeners.begin(); it != mUpdateListeners.end(); it++)
        (*it)->objectBoundsUpdated(this, old_bounds, new_bounds);
}

void Object::addUpdateListener(ObjectUpdateListener* listener) {
    assert(listener != NULL);
    assert(std::find(mUpdateListeners.begin(), mUpdateListeners.end(), listener) == mUpdateListeners.end());

    mUpdateListeners.push_back(listener);
}

void Object::removeUpdateListener(ObjectUpdateListener* listener) {
    assert(listener != NULL);

    UpdateListenerList::iterator it = std::find(mUpdateListeners.begin(), mUpdateListeners.end(), listener);
    if (it == mUpdateListeners.end())
        return;

    mUpdateListeners.erase(it);
}

} // namespace Simulation
} // namespace Prox
