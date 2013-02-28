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
//
// Returns -1 if the object doesn't satisfy the query, the computed solid angle
// if it does
template<typename SimulationTraits>
float32 satisfiesConstraintsBounds(
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

    // Must satisfy solid angle constraint
    // If it falls inside the query bounds, then it definitely satisfies the solid angle constraint
    // FIXME we do this check manually for now, but BoundingSphere should
    // provide it
    Vector3 to_obj = obj_pos - query_pos;
    float worst_query_obj_radius = query_rad + obj_radius;
    if (worst_query_obj_radius*worst_query_obj_radius >= to_obj.lengthSquared()) {
        return SolidAngle::Max.asFloat();
    }
    // Otherwise we need to check the closest possible query position to the object

    // Could compute the actual, final vector to the object:
    //   to_obj = to_obj - to_obj.normal()*query_rad;
    // but we only need the distance. Unfortunately we can't avoid the sqrt
    // here.
    float to_obj_dist = to_obj.length() - query_rad;
    float to_obj_dist_sq = to_obj_dist*to_obj_dist;

    // Must satisfy radius constraint
    if (qradius != SimulationTraits::InfiniteRadius && to_obj.lengthSquared() > qradius*qradius)
        return -1;

    // Must satisfy solid angle constraint. lessThanEqualDistanceSqRadius gets us
    // the right value -- -1 if false, positive score for the standin if true
    return qangle.lessThanEqualDistanceSqRadius(to_obj_dist_sq, obj_radius);
}

// Check if an object satisfies the constraints of a query using bounds and max
// size information.
//
// For this version, since the region and maximum object size are separate, the
// region should be determined only using the objects' centers.
//
// Returns -1 if the object doesn't satisfy the query, the computed solid angle
// if it does
template<typename SimulationTraits>
float32 satisfiesConstraintsBoundsAndMaxSize(
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
    Vector3 to_obj = obj_pos - query_pos;
    float worst_query_obj_radius = query_rad + obj_radius + obj_max_size;
    if (worst_query_obj_radius*worst_query_obj_radius >= to_obj.lengthSquared())
        return SolidAngle::Max.asFloat();

    float standin_radius = obj_max_size;

    // Could compute the actual, final vector to the object:
    //   to_obj = to_obj - to_obj.normal()*worst_query_obj_radius;
    // but we only need the distance. Unfortunately we can't avoid the sqrt
    // here.
    float to_obj_dist = to_obj.length() - worst_query_obj_radius;
    float to_obj_dist_sq = to_obj_dist*to_obj_dist;

    // Must satisfy radius constraint
    if (qradius != SimulationTraits::InfiniteRadius && to_obj_dist_sq > qradius*qradius)
        return -1;

    // Must satisfy solid angle constraint. lessThanEqualDistanceSqRadius gets us
    // the right value -- -1 if false, positive score for the standin if true
    return qangle.lessThanEqualDistanceSqRadius(to_obj_dist_sq, standin_radius);
}

} // namespace Prox

#endif //_PROX_CONSTRAINTS_HPP_
