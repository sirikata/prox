/*  libprox
 *  Constraints.hpp
 *
 *  Copyright (c) 2010, Ewen Cheslack-Postava
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

#ifndef _PROX_CONSTRAINTS_HPP_
#define _PROX_CONSTRAINTS_HPP_

namespace Prox {

// Check if an object satisfies the constraints of a query using only bounds
// information.
//
// For this version, the bounds (obj_radius) should be the bounds of all the
// *actual objects* not just their centers.
template<typename SimulationTraits>
bool satisfiesConstraintsBounds(
    const typename SimulationTraits::Vector3Type& obj_pos,
    float obj_radius,
    const typename SimulationTraits::Vector3Type& qpos,
    const typename SimulationTraits::BoundingSphereType& qbounds,
    const float qmaxsize,
    const typename SimulationTraits::SolidAngleType& qangle,
    const float qradius)
{
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;

    // Combine info to reduce amount we need to deal with
    Vector3 query_pos = qpos + qbounds.center();
    // Just sum these since we don't take the solid angle of these, so it is
    // equivalent to extending the radius by the size of the largest object.
    float query_rad = qbounds.radius() + qmaxsize;

    // Must satisfy radius constraint
    if (qradius != SimulationTraits::InfiniteRadius && (obj_pos-query_pos).lengthSquared() > qradius*qradius)
        return false;

    // Must satisfy solid angle constraint
    // If it falls inside the query bounds, then it definitely satisfies the solid angle constraint
    // FIXME we do this check manually for now, but BoundingSphere should provide it
    if (query_rad + obj_radius >= (query_pos-obj_pos).length()) {
        return true;
    }
    // Otherwise we need to check the closest possible query position to the object
    Vector3 to_obj = obj_pos - query_pos;
    to_obj = to_obj - to_obj.normal() * query_rad;
    SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, obj_radius);

    if (solid_angle >= qangle)
        return true;

    return false;
}

// Check if an object satisfies the constraints of a query using bounds and max
// size information.
//
// For this version, since the region and maximum object size are separate, the
// region should be determined only using the objects' centers.
template<typename SimulationTraits>
bool satisfiesConstraintsBoundsAndMaxSize(
    const typename SimulationTraits::Vector3Type& obj_pos,
    float obj_radius,
    float obj_max_size,
    const typename SimulationTraits::Vector3Type& qpos,
    const typename SimulationTraits::BoundingSphereType& qbounds,
    const float qmaxsize,
    const typename SimulationTraits::SolidAngleType& qangle,
    const float qradius)
{
    typedef typename SimulationTraits::Vector3Type Vector3;
    typedef typename SimulationTraits::BoundingSphereType BoundingSphere;
    typedef typename SimulationTraits::SolidAngleType SolidAngle;

    // We create a virtual stand in object which is the worst case object that could be in this subtree.
    // It's centered at the closest point on the hierarchical bounding sphere to the query, and has the
    // largest radius of any objects in the subtree.

    // Combine info to reduce amount we need to deal with
    Vector3 query_pos = qpos + qbounds.center();
    // Just sum these since we don't take the solid angle of these, so it is
    // equivalent to extending the radius by the size of the largest object.
    float query_rad = qbounds.radius() + qmaxsize;

    // First, a special case is if the query is actually inside the hierarchical bounding sphere, in which
    // case the above description isn't accurate: in this case the worst position for the stand in object
    // is the exact location of the query.  So just let it pass.
    // FIXME we do this check manually for now, but BoundingSphere should provide it
    if (query_rad + obj_radius + obj_max_size >= (query_pos-obj_pos).length())
        return true;

    float standin_radius = obj_max_size;

    Vector3 to_obj = obj_pos - query_pos;
    to_obj = to_obj - to_obj.normal() * (obj_radius + obj_max_size + query_rad);

    // Must satisfy radius constraint
    if (qradius != SimulationTraits::InfiniteRadius && to_obj.lengthSquared() > qradius*qradius)
        return false;

    // Must satisfy solid angle constraint
    SolidAngle solid_angle = SolidAngle::fromCenterRadius(to_obj, standin_radius);

    if (solid_angle >= qangle)
        return true;

    return false;
}

} // namespace Prox

#endif //_PROX_CONSTRAINTS_HPP_
