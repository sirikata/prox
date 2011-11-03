// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_SIMULATOR_OBJECT_LISTENER_HPP_
#define _PROXSIM_SIMULATOR_OBJECT_LISTENER_HPP_

#include "Object.hpp"

namespace Prox {
namespace Simulation {

class SimulatorObjectListener {
public:
    SimulatorObjectListener() {}
    virtual ~SimulatorObjectListener() {}

    virtual void simulatorAddedObject(Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) = 0;
    virtual void simulatorRemovedObject(Object* obj) = 0;
}; // class SimulatorObjectListener

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_SIMULATOR_OBJECT_LISTENER_HPP_
