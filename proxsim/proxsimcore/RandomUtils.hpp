// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIMCORE_RANDOMUTILS_HPP_
#define _PROXSIMCORE_RANDOMUTILS_HPP_

#include "MotionPath.hpp"

namespace Prox {
namespace Simulation {

inline float randFloat() {
    return float(rand()) / RAND_MAX;
}

inline uint32 randUInt32(uint32 minval, uint32 maxval) {
    uint32 r = (uint32)( randFloat() * (maxval-minval+1) + minval);
    if (r > maxval) r = maxval;
    return r;
}

inline Vector3 generatePosition(const BoundingBox3& region) {
    Vector3 region_min = region.min();
    Vector3 region_extents = region.extents();

    return region_min + Vector3(region_extents.x * randFloat(), region_extents.y * randFloat(), 0.f/*region_extents.z * randFloat()*/);
}

inline Vector3 generateDirection(bool moving) {
    return
        (moving ?
            Vector3(randFloat() * 20.f - 10.f, randFloat() * 20.f - 10.f, 0.f/*randFloat() * 20.f - 10.f*/) :
            Vector3(0, 0, 0)
        );
};

inline MotionPath generateMotionPath(const BoundingBox3& region, bool moving) {
    // Our simple model is ballistic, single update

    Vector3 offset = generatePosition(region);
    MotionPath::MotionVectorListPtr updates(
        new MotionPath::MotionVectorList()
    );
    Vector3 dir = generateDirection(moving);
    updates->push_back(
        MotionVector3(
            Time::null(),
            Vector3(0,0,0),
            dir
        )
    );

    return MotionPath(offset, updates);
}

inline BoundingBox3 generateObjectBounds() {
    return BoundingBox3( Vector3(-1, -1, -1), Vector3(1, 1, 1));
};

inline BoundingBox3 generateQueryBounds() {
    return BoundingBox3( Vector3(0, 0, 0), Vector3(0, 0, 0) );
}

inline float generateQueryRadius() {
    static float val = sqrtf(6.f) / 2.f; // Diagonal of bounding box
    return val;
}

inline SolidAngle generateQueryAngle(const SolidAngle& qmin, const SolidAngle& qmax) {
    assert(qmax >= qmin);
    if (qmax == qmin) return qmin;

    return qmin + ((qmax-qmin) * (((float)(rand()))/RAND_MAX));
}

} // namespace Simulation
} // namespace Prox

#endif // _PROXSIMCORE_RANDOMUTILS_HPP_
