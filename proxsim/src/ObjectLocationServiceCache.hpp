/*  proxsim
 *  ObjectLocationServiceCache.hpp
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

#ifndef _PROX_OBJECT_LOCATION_SERVICE_CACHE_HPP_
#define _PROX_OBJECT_LOCATION_SERVICE_CACHE_HPP_

#include "Object.hpp"

namespace Prox {
namespace Simulation {

/* Implementation of LocationServiceCache which deals directly with locally
 * simulated objects.
 */
class ObjectLocationServiceCache : public LocationServiceCache, public ObjectUpdateListener {
public:
    ObjectLocationServiceCache();
    virtual ~ObjectLocationServiceCache();

    void addObject(Object* obj);
    void removeObject(const Object* obj);

    virtual void startTracking(const ObjectID& id);
    virtual void stopTracking(const ObjectID& id);

    virtual const MotionVector3& location(const ObjectID& id) const;
    virtual const BoundingSphere& bounds(const ObjectID& id) const;

    virtual void addUpdateListener(LocationUpdateListenerType* listener);
    virtual void removeUpdateListener(LocationUpdateListenerType* listener);

    virtual void objectPositionUpdated(Object* obj, const MotionVector3& old_pos, const MotionVector3& new_pos);
    virtual void objectBoundsUpdated(Object* obj, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds);
    virtual void objectDeleted(const Object* obj);

private:
    Object* lookup(const ObjectID& id) const;

    typedef std::map<ObjectID, Object*> ObjectMap;
    typedef std::set<LocationUpdateListenerType*> ListenerSet;

    ObjectMap mObjects;
    ListenerSet mListeners;
};

} // namespace Simulation
} // namespace Prox

#endif //_PROX_OBJECT_LOCATION_SERVICE_CACHE_HPP_
