/*  libprox
 *  LocationServiceCache.hpp
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

#ifndef _PROX_LOCATION_SERVICE_CACHE_HPP_
#define _PROX_LOCATION_SERVICE_CACHE_HPP_

#include <prox/util/Platform.hpp>
#include <prox/base/LocationUpdateListener.hpp>
#include <prox/base/ZernikeDescriptor.hpp>

namespace Prox {

/* Abstract base class for a cache which sits in front of a LocationService.
 * The cache should guarantee that requests for location information can be
 * answered immediately for objects for which caching is requested.  This
 * guarantees that clients of the LocationService will always be able to get
 * the necessary information out of it, even if it is somewhat out of date.
 * Note that LocationServiceCaches must be thread-safe.
 */
template<typename SimulationTraits>
class LocationServiceCache : public LocationUpdateProvider<SimulationTraits> {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::TimeType Time;
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    /** An Iterator is an opaque reference to an entry in the cache. Starting
     *  tracking returns an iterator which is used to efficiently access data
     *  for that object.
     */
    class Iterator {
    public:
        Iterator() : data(NULL) {};
        explicit Iterator(void* _data) : data(_data) {};
        void* data;

        bool operator==(const Iterator& rhs) const {
            return (data == rhs.data);
        }
    };

    virtual ~LocationServiceCache() {}


    // Since LocationServiceCaches are thread-safe, but LocationServices may
    // not be, we can get into a weird situation where one thread knows about a
    // new, internally-generated aggregate, e.g. because a node was added to a
    // query data structure, but it hasn't been added to the
    // LocationServiceCache because it needs to be routed through the
    // LocationService first. To address this, LocationServiceCaches must be
    // able to add temporary placeholders immediately with their basic
    // properties, but set them up in a way such that they can be cleaned up
    // normally, i.e. the forthcoming addition and following removal will be
    // handled as if this call never happened.
    //
    // Note that this is called addPlaceholderImposter, not
    // addPlaceholder. There should never be a reason to call it with real
    // objects, which should make it through the LocationService normally; only
    // imposter (aggregates/bogus objects) should need to be inserted
    // immediately as they are created dynamically. This also means that we
    // don't pass in all the information that a LocationServiceCache may contain
    // for all objects -- the implementation is responsible for filling in any
    // reasonable default values.
    virtual void addPlaceholderImposter(
        const ObjectID& id,
        const Vector3& center_pos,
        const float32 center_bounds_radius,
        const float32 max_size,
        const String& zernike,
        const String& mesh
    ) = 0;


    /** Start tracking the object with the given ID and return an
     *  Iterator which can be used to access its data. You must call
     *  stopTracking(iterator) to allow the object to be freed when
     *  you are done with it.
     */
    virtual Iterator startTracking(const ObjectID& id) = 0;
    /** Stop tracking the object with the given iterator obtained from
     *  startTracking.
     */
    virtual void stopTracking(const Iterator& id) = 0;
    /** Start tracking an object by just increasint its refcount. This
     *  doesn't give access to it's data but does ensure the entry
     *  remains in the cache. This must be accompanied by a later call
     *  to stopRefcountTracking.
     */
    virtual bool startRefcountTracking(const ObjectID& id) = 0;
    /** Stop tracking an object by decreasing its refcount. */
    virtual void stopRefcountTracking(const ObjectID& id) = 0;

    // The various properties maintained here can get a bit confusing because
    // they are designed to support a few different scenarios.
    //
    // Location is used to specify the center point of the object(s) being
    // represented.  It is the only way to introduce movement into the system:
    // it depends on a time parameter.
    //
    // Region represents the region covered by the *locations* of the
    // constituent objects in the group.  Note that this is not the same as the
    // region covered by the objects in the group: it is smaller for
    // non-zero-sized objects.  This can be computed without any of the object
    // sizes. For a single object, this will be degenerate since it only
    // includes the central location.
    //
    // For multiple objects, location and region should be computed together and
    // the region's center should be the origin. This results in a smaller
    // covered region than computing them independently.
    //
    // MaxSize is the size of the largest object contained in this set of objects.
    // It can be computed using only the objects bounding regions.  For a single
    // object this is just the size of the object.
    //
    //
    // The helper method worldRegion simply computes the current combination of
    // location and region, i.e. the world space bounding sphere of the
    // locations of all objects in the region.
    //
    // The helper method worldCompleteBounds computes a bounding sphere for the
    // entire contents of the element (worldRegion + maxSize to radius).

    virtual MotionVector3 location(const Iterator& id) = 0;
    // These are the core values that describe the bounds of the
    // object/aggregate:
    //  Offset of the center from the location -- a fixed value. We actually
    //  wouldn't care if this is combined with location() because it only affects
    //  rotations.
    virtual Vector3 centerOffset(const Iterator& id) = 0;
    //  The radius of the bounding sphere that encompasses the *center points*
    //  of all child objects
    virtual float32 centerBoundsRadius(const Iterator& id) = 0;
    //  The maximum size of any of the children
    virtual float32 maxSize(const Iterator& id) = 0;
    // And these are derived values
    virtual BoundingSphere region(const Iterator& id) {
        return BoundingSphere(centerOffset(id), centerBoundsRadius(id));
    }
    virtual BoundingSphere worldRegion(const Iterator& id, const Time& t) {
        BoundingSphere reg = region(id);
        return BoundingSphere( reg.center() + location(id).position(t), reg.radius() );
    }
    virtual BoundingSphere worldCompleteBounds(const Iterator& id, const Time& t) {
        BoundingSphere reg = region(id);
        return BoundingSphere( reg.center() + location(id).position(t), reg.radius() + maxSize(id) );
    }

    virtual ZernikeDescriptor& zernikeDescriptor(const Iterator& id) = 0;
    virtual String mesh(const Iterator& id) = 0;

    // Returns true if the object is local, false if it is a replica
    virtual bool isLocal(const Iterator& id) = 0;

    virtual const ObjectID& iteratorID(const Iterator& id) = 0;

};

} // namespace Prox

#endif //_PROX_LOCATION_SERVICE_CACHE_HPP_
