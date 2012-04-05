/*  libprox
 *  LocationUpdateListener.hpp
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

#ifndef _PROX_OBJECT_CHANGE_LISTENER_HPP_
#define _PROX_OBJECT_CHANGE_LISTENER_HPP_

namespace Prox {

template<typename SimulationTraits>
class LocationUpdateListener {
public:
    typedef typename SimulationTraits::realType Real;
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::MotionVector3Type MotionVector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;

    LocationUpdateListener() {}
    virtual ~LocationUpdateListener() {}

    virtual void locationConnected(const ObjectID& obj_id, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) = 0;
    virtual void locationConnectedWithParent(const ObjectID& obj_id, const ObjectID& parent, bool local, const MotionVector3& pos, const BoundingSphere& region, Real maxSize) = 0;
    virtual void locationParentUpdated(const ObjectID& obj_id, const ObjectID& old_par, const ObjectID& new_par) = 0;
    virtual void locationPositionUpdated(const ObjectID& obj_id, const MotionVector3& old_pos, const MotionVector3& new_pos) = 0;
    virtual void locationRegionUpdated(const ObjectID& obj_id, const BoundingSphere& old_region, const BoundingSphere& new_region) = 0;
    virtual void locationMaxSizeUpdated(const ObjectID& obj_id, Real old_maxSize, Real new_maxSize) = 0;
    // The temporary flag allows you to override how the removal would otherwise
    // be viewed. By default, the removal is permanent, but there may be cases
    // where you want to indicate that it is actually temporary (e.g. because
    // your LocationServiceCache is fed by proximity results, which want to mask
    // quick additions/failures).
    virtual void locationDisconnected(const ObjectID& obj_id, bool temporary = false) = 0;

}; // class LocationUpdateListener

template<typename SimulationTraits>
class LocationUpdateProvider {
public:
    typedef LocationUpdateListener<SimulationTraits> LocationUpdateListenerType;

    virtual ~LocationUpdateProvider() {}

    virtual void addUpdateListener(LocationUpdateListenerType* listener) = 0;
    virtual void removeUpdateListener(LocationUpdateListenerType* listener) = 0;
};

} // namespace Prox

#endif //_PROX_OBJECT_CHANGE_LISTENER_HPP_
