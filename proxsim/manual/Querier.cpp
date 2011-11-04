// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "Querier.hpp"

namespace Prox {
namespace Simulation {

Querier::Querier(ManualQueryHandler* handler, const MotionPath& mp, const BoundingSphere& bounds, float max_querier_radius)
 : mMotion(mp)
{
    mQuery = handler->registerQuery(
        mp.current(),
        bounds, max_querier_radius
    );
}

Querier::~Querier() {
    delete mQuery;
}

void Querier::tick(const Time& t) {
    bool changed = mMotion.tick(t);
    if (changed)
        mQuery->position(mMotion.current());
}

} // namespace Simulation
} // namespace Prox
