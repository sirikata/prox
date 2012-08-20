// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_LOCATION_SERVICE_CACHE_HPP_
#define _LIBPROX_TEST_LOCATION_SERVICE_CACHE_HPP_

#include <prox/base/LocationServiceCache.hpp>
#include <prox/base/DefaultSimulationTraits.hpp>

/* Implementation of LocationServiceCache which deals directly with locally
 * simulated objects.
 */
class TestLocationServiceCache : public Prox::LocationServiceCache<Prox::DefaultSimulationTraits> {
public:
    typedef Prox::float32 float32;
    typedef Prox::String String;
    typedef Prox::ZernikeDescriptor ZernikeDescriptor;

    TestLocationServiceCache();
    virtual ~TestLocationServiceCache();

    // External data input.
    // The notify paremeter gives control of notifying listeners,
    // i.e. handlers. This lets you, e.g., add an aggregate but not notify the
    // handler since it would just try to add it.
    void addObject(const ObjectID& objid,
        bool aggregate,
        const MotionVector3& loc,
        const Vector3& bounds_center_offset,
        const float32 bounds_center_bounds_radius,
        const float32 bounds_max_size,
        const String& mesh,
        bool notify);
    void addObjectWithParent(const ObjectID& objid,
        const ObjectID& parentid,
        bool aggregate,
        const MotionVector3& loc,
        const Vector3& bounds_center_offset,
        const float32 bounds_center_bounds_radius,
        const float32 bounds_max_size,
        const String& mesh,
        bool notify);
    void removeObject(const ObjectID& objid);
    void updateLocation(const ObjectID& objid, const MotionVector3& newval);
    void updateBounds(const ObjectID& objid,
        const Vector3& bounds_center_offset,
        const float32 bounds_center_bounds_radius,
        const float32 bounds_max_size);
    void updateMesh(const ObjectID& objid, const String& newval);

    virtual void addPlaceholderImposter(const ObjectID& id,
        const Vector3& center_offset,
        const float32 center_bounds_radius,
        const float32 max_size,
        const String& zernike,
        const String& mesh
    );

    virtual Iterator startTracking(const ObjectID& id);
    virtual void stopTracking(const Iterator& id);
    virtual bool startRefcountTracking(const ObjectID& id);
    virtual void stopRefcountTracking(const ObjectID& id);

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

private:

    struct ObjectInfo {
        ObjectID objid;
        bool aggregate;
        MotionVector3 loc;
        Vector3 bounds_center_offset;
        float32 bounds_center_bounds_radius;
        float32 bounds_max_size;
        String mesh;

        bool exists;
        int refcount;

        ObjectInfo()
         : objid(ObjectID::null()),
           loc(Time::null(), Vector3::nil(), Vector3::nil()),
           exists(false), refcount(0)
        {}
        ObjectInfo(const ObjectID& _objid, bool _aggregate,
            const MotionVector3& _loc,
            const Vector3& _bounds_center_offset,
            const float32 _bounds_center_bounds_radius,
            const float32 _bounds_max_size,
            const String& _mesh)
         : objid(_objid),
           aggregate(_aggregate),
           loc(_loc),
           bounds_center_offset(_bounds_center_offset),
           bounds_center_bounds_radius(_bounds_center_bounds_radius),
           bounds_max_size(_bounds_max_size),
           mesh(_mesh),
           exists(true), refcount(0)
        {}
    };
    typedef std::tr1::shared_ptr<ObjectInfo> ObjectInfoPtr;

    typedef std::tr1::unordered_map<ObjectID, ObjectInfoPtr, ObjectID::Hasher> ObjectMap;
    typedef std::tr1::unordered_set<LocationUpdateListenerType*> ListenerSet;

    void tryClearObject(ObjectMap::iterator& it);

    ObjectMap mObjects;
    ListenerSet mListeners;
};

#endif //_LIBPROX_TEST_LOCATION_SERVICE_CACHE_HPP_
