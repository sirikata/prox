/*  libprox
 *  BoundingSphere.hpp
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

#ifndef _BOUNDING_SPHERE_HPP_
#define _BOUNDING_SPHERE_HPP_

#include <prox/Platform.hpp>
#include <cmath>

namespace Prox {
namespace Reference {

class BoundingSphereBase {
public:
    static const double Pi;
};

template<typename CoordType>
class BoundingSphere : public BoundingSphereBase {
public:
    typedef typename CoordType::real real;

    BoundingSphere()
     : mCenter((real)0),
       mRadius(-1.0)
    {
        mRadius = -1.0;
    }

    BoundingSphere(const CoordType& c, const real r)
     : mCenter(c),
       mRadius(r)
    {
    }

    const CoordType& center() const {
        return mCenter;
    }

    real radius() const {
        return mRadius;
    }

    BoundingSphere& mergeIn(const BoundingSphere& rhs) {
        *this = merge(rhs);
        return *this;
    }

    BoundingSphere merge(const BoundingSphere& rhs) const {
        if (rhs.invalid())
            return *this;

        if (this->invalid())
            return rhs;

        // Check if one is entirely contained within the other
        CoordType to_other_center = rhs.mCenter - mCenter;
        real center_dist = to_other_center.length();
        if (center_dist + mRadius <= rhs.mRadius)
            return rhs;
        if (center_dist + rhs.mRadius <= mRadius)
            return *this;

        real new_radius2 = (mRadius + center_dist + rhs.mRadius);
        real new_radius = new_radius2 * 0.5;
        if (center_dist > 1e-08) {
            CoordType to_other_center_normalized = to_other_center / center_dist;
            CoordType farthest_point_from_other = mCenter - (mRadius * to_other_center_normalized);
            CoordType half_new_span = to_other_center_normalized * new_radius;
            CoordType new_center = farthest_point_from_other + half_new_span;
            return BoundingSphere(new_center, new_radius);
        }
        else {
            return BoundingSphere(mCenter, new_radius);
        }
    }

    bool contains(const BoundingSphere& other) const {
        real centers_len = (mCenter - other.mCenter).length();
        return (mRadius >= centers_len + other.mRadius);
    }

    bool contains(const BoundingSphere& other, real epsilon) const {
        real centers_len = (mCenter - other.mCenter).length();
        return (mRadius + epsilon >= centers_len + other.mRadius);
    }

    bool contains(const CoordType& pt) const {
        return ( (mCenter-pt).lengthSquared() <= mRadius*mRadius );
    }

    bool invalid() const {
        return ( mRadius < 0 );
    }

    bool degenerate() const {
        return ( mRadius <= 0 );
    }

    real volume() const {
        if (degenerate()) return 0.0;
        return 4.0 / 3.0 * Pi * mRadius * mRadius * mRadius;
    }

    bool operator==(const BoundingSphere& rhs) {
        return (mCenter == rhs.mCenter && mRadius == rhs.mRadius);
    }
    bool operator!=(const BoundingSphere& rhs) {
        return (mCenter != rhs.mCenter || mRadius != rhs.mRadius);
    }
private:
    CoordType mCenter;
    real mRadius;
}; // class BoundingSphere


template<typename scalar>
class Vector3;

typedef BoundingSphere< Vector3<float32> > BoundingSphere3f;
typedef BoundingSphere< Vector3<float64> > BoundingSphere3d;

} // namespace Reference
} // namespace Prox

#endif //_BOUNDING_SPHERE_HPP_
