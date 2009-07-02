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

#include <prox/Platform.hpp>
#include <prox/LocationUpdateListener.hpp>

namespace Prox {

/* Abstract base class for a cache which sits in front of a LocationService.
 * The cache should guarantee that requests for location information can be
 * answered immediately for objects for which caching is requested.  This
 * guarantees that clients of the LocationService will always be able to get
 * the necessary information out of it, even if it is somewhat out of date.
 */
template<typename SimulationTraits>
class LocationServiceCache {
public:
    typedef typename SimulationTraits::ObjectID ObjectID;
    typedef typename SimulationTraits::Time Time;
    typedef typename SimulationTraits::MotionVector3 MotionVector3;
    typedef typename SimulationTraits::BoundingSphere BoundingSphere;
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListener;

    virtual ~LocationServiceCache() {}

    virtual void startTracking(const ObjectID& id) = 0;
    virtual void stopTracking(const ObjectID& id) = 0;

    virtual const MotionVector3& location(const ObjectID& id) const = 0;
    virtual const BoundingSphere& bounds(const ObjectID& id) const = 0;
    virtual BoundingSphere worldBounds(const ObjectID& id, const Time& t) const {
        return BoundingSphere( bounds(id).center() + location(id).position(t), bounds(id).radius() );
    }

    virtual void addUpdateListener(LocationUpdateListener* listener) = 0;
    virtual void removeUpdateListener(LocationUpdateListener* listener) = 0;
};

} // namespace Prox

#endif //_PROX_LOCATION_SERVICE_CACHE_HPP_
