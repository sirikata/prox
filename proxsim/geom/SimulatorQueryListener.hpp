// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_SIMULATOR_QUERY_LISTENER_HPP_
#define _PROXSIM_SIMULATOR_QUERY_LISTENER_HPP_

#include "Querier.hpp"

namespace Prox {
namespace Simulation {

class SimulatorQueryListener {
public:
    SimulatorQueryListener() {}
    virtual ~SimulatorQueryListener() {}

    virtual void simulatorAddedQuery(Querier* query) = 0;
    virtual void simulatorRemovedQuery(Querier* query) = 0;
}; // class SimulatorQueryListener

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_SIMULATOR_QUERY_LISTENER_HPP_
