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
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    /** An Iterator is an opaque reference to an entry in the cache. Starting
     *  tracking returns an iterator which is used to efficiently access data
     *  for that object.
     */
    class Iterator {
    public:
        Iterator() : data(NULL) {};
        Iterator(void* _data) : data(_data) {};
        void* data;

        bool operator==(const Iterator& rhs) const {
            return (data == rhs.data);
        }
    };

    virtual ~LocationServiceCache() {}

    virtual Iterator startTracking(const ObjectID& id) = 0;
    virtual void stopTracking(const Iterator& id) = 0;

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
    virtual BoundingSphere region(const Iterator& id) = 0;
    virtual BoundingSphere worldRegion(const Iterator& id, const Time& t) {
        BoundingSphere reg = region(id);
        return BoundingSphere( reg.center() + location(id).position(t), reg.radius() );
    }
    virtual float32 maxSize(const Iterator& id) = 0;
    virtual BoundingSphere worldCompleteBounds(const Iterator& id, const Time& t) {
        BoundingSphere reg = region(id);
        return BoundingSphere( reg.center() + location(id).position(t), reg.radius() + maxSize(id) );
    }
    // Returns true if the object is local, false if it is a replica
    virtual bool isLocal(const Iterator& id) = 0;

    virtual const ObjectID& iteratorID(const Iterator& id) = 0;

};

} // namespace Prox

#endif //_PROX_LOCATION_SERVICE_CACHE_HPP_
