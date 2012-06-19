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
#include "SimulatorObjectListener.hpp"
#include <boost/thread.hpp>

namespace Prox {
namespace Simulation {

/* Implementation of LocationServiceCache which deals directly with locally
 * simulated objects.
 */
class ObjectLocationServiceCache : public LocationServiceCache, public ObjectUpdateListener, public SimulatorObjectListener {
public:
    ObjectLocationServiceCache();
    virtual ~ObjectLocationServiceCache();

    void addObject(Object* obj);
    void removeObject(const Object* obj);

    virtual void addPlaceholderImposter(const ObjectID& id,
        const Vector3& center_offset,
        const float32 center_bounds_radius,
        const float32 max_size,
        const String& zernike,
        const String& mesh
    );

    virtual Iterator startTracking(const ObjectID& id);
    virtual void stopTracking(const Iterator& id);

    virtual MotionVector3 location(const Iterator& id);
    virtual Vector3 centerOffset(const Iterator& id);
    virtual float32 centerBoundsRadius(const Iterator& id);
    virtual float32 maxSize(const Iterator& id);
    virtual ZernikeDescriptor& zernikeDescriptor(const Iterator& id);
    virtual String mesh(const Iterator& id);
    virtual bool isLocal(const Iterator& id);

    virtual const ObjectID& iteratorID(const Iterator& id);

    virtual void addUpdateListener(LocationUpdateListenerType* listener);
    virtual void removeUpdateListener(LocationUpdateListenerType* listener);

    // ObjectUpdateListener Methods
    virtual void objectCreated(const Object* obj, const MotionVector3& pos, const BoundingSphere& bounds);
    virtual void objectPositionUpdated(Object* obj, const MotionVector3& old_pos, const MotionVector3& new_pos);
    virtual void objectBoundsUpdated(Object* obj, const BoundingSphere& old_bounds, const BoundingSphere& new_bounds);
    virtual void objectDeleted(const Object* obj);

    // SimulatorObjectListener Methods
    // Note: We'd prefer to do this just with objects but there isn't a good way to set up the listeners ahead
    // of time, so we need to use the simulator just to get the object additions
    virtual void simulatorAddedObject(Object* obj, const MotionVector3& pos, const BoundingSphere& bounds);
    virtual void simulatorRemovedObject(Object* obj);
private:
    void tryClearObject(const Object* obj);

    static BoundingSphere sNullBoundingSphere;

    struct ObjectInfo {
        Object* object;
        bool exists;
        int refcount;

        ObjectInfo() : object(NULL), exists(false), refcount(0) {}
        ObjectInfo(Object* obj) : object(obj), exists(true), refcount(0) {}
    };

    typedef std::tr1::unordered_map<ObjectID, ObjectInfo, ObjectID::Hasher> ObjectMap;
    typedef std::tr1::unordered_set<LocationUpdateListenerType*> ListenerSet;

    typedef boost::recursive_mutex Mutex;
    typedef boost::lock_guard<Mutex> Lock;
    Mutex mMutex;

    ObjectMap mObjects;
    ListenerSet mListeners;
};

} // namespace Simulation
} // namespace Prox

#endif //_PROX_OBJECT_LOCATION_SERVICE_CACHE_HPP_
