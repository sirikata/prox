// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_CSVLOADER_HPP_
#define _PROXSIM_MANUAL_CSVLOADER_HPP_

#include "Querier.hpp"

namespace Prox {
namespace Simulation {

std::vector<Querier*> loadCSVMotionQueriers(const String& filename, int nqueriers, ManualQueryHandler* qh, std::tr1::function<Vector3()> gen_loc, float qradius);

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_CSVLOADER_HPP_
