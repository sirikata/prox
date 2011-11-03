// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_SIMULATOR_HPP_
#define _PROXSIM_MANUAL_SIMULATOR_HPP_

#include <proxsimcore/SimulatorBase.hpp>

namespace Prox {
namespace Simulation {

class Simulator : public SimulatorBase {
public:
    Simulator(ManualQueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime);
    ~Simulator();

    void initialize(int churnrate);

    virtual void shutdown();

    virtual void tick_work(Time last_time, Duration elapsed);

private:
    ManualQueryHandler* mHandler;
}; // class Simulator

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_SIMULATOR_HPP_
