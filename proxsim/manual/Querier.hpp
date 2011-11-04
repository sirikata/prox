// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_QUERIER_HPP_
#define _PROXSIM_MANUAL_QUERIER_HPP_

#include <proxsimcore/SimulationTypes.hpp>
#include <proxsimcore/MotionPath.hpp>

namespace Prox {
namespace Simulation {

class Querier {
public:
    Querier(ManualQueryHandler* handler, const MotionPath& mp, const BoundingSphere& bounds, float max_querier_radius);
    ~Querier();

    void tick(const Time& t);

    // Pass through calls for Query
    void setEventListener(ManualQueryEventListener* listener) {
        mQuery->setEventListener(listener);
    }
    Vector3 position(const Time& t) const {
        return mQuery->position(t);
    }
private:
    MotionPath mMotion;
    ManualQuery* mQuery;
};

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_QUERIER_HPP_
