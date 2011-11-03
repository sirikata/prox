// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "Simulator.hpp"

namespace Prox {
namespace Simulation {

Simulator::Simulator(ManualQueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime)
 : SimulatorBase(duration, timestep, iterations, realtime),
   mHandler(handler)
{
}

Simulator::~Simulator() {
}

void Simulator::initialize(int churnrate) {
    SimulatorBase::initialize(churnrate);
    mHandler->initialize(mLocCache, mLocCache, !mHaveMovingObjects);
}

void Simulator::shutdown() {
    delete mHandler;
    mHandler = NULL;

    SimulatorBase::shutdown();
}

void Simulator::tick_work(Time last_time, Duration elapsed) {
    SimulatorBase::tick_work(last_time, elapsed);

    mHandler->tick(mTime);
}

} // namespace Simulation
} // namespace Prox
