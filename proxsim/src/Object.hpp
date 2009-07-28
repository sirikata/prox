/*  proxsim
 *  Object.hpp
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

#ifndef _PROX_OBJECT_HPP_
#define _PROX_OBJECT_HPP_

#include "SimulationTypes.hpp"

namespace Prox {
namespace Simulation {

class ObjectUpdateListener;

class Object {
public:
    Object(const ObjectID& id, const MotionVector3& c, const BoundingSphere& bs);
    Object(const Object& cpy);
    ~Object();

    const ObjectID& id() const;
    const MotionVector3& position() const;
    Vector3 position(const Time& t) const;
    const BoundingSphere& bounds() const;
    BoundingSphere worldBounds(const Time& t) const;
    void position(const MotionVector3& new_pos);
    void bounds(const BoundingSphere& new_bounds);

    void addUpdateListener(ObjectUpdateListener* listener);
    void removeUpdateListener(ObjectUpdateListener* listener);
protected:

    ObjectID mID;
    MotionVector3 mPosition;
    BoundingSphere mBounds;

    typedef std::list<ObjectUpdateListener*> UpdateListenerList;
    UpdateListenerList mUpdateListeners;
private:
    Object();
}; // class Object

class ObjectUpdateListener {
public:
    ObjectUpdateListener() {}
    virtual ~ObjectUpdateListener() {}

    virtual void objectCreated(const Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) = 0;
    virtual void objectPositionUpdated(Object* obj, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void objectBoundsUpdated(Object* obj, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds) = 0;
    virtual void objectDeleted(const Object* obj) = 0;
}; // class ObjectUpdateListener

} // namespace Simulation
} // namespace Prox

#endif //_PROX_OBJECT_HPP_
