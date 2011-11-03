// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "Simulator.hpp"

namespace Prox {
namespace Simulation {

Simulator::Simulator(int duration, const Duration& timestep, int iterations, bool realtime)
 : SimulatorBase(duration, timestep, iterations, realtime)
{
}

Simulator::~Simulator() {
}

void Simulator::initialize(int churnrate) {
    SimulatorBase::initialize(churnrate);
}

void Simulator::shutdown() {
    SimulatorBase::shutdown();
}

void Simulator::tick_work(Time last_time, Duration elapsed) {
    SimulatorBase::tick_work(last_time, elapsed);
}

} // namespace Simulation
} // namespace Prox
