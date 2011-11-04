// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "CSVLoader.hpp"
#include <proxsimcore/CSVLoader.hpp>

namespace Prox {
namespace Simulation {

std::vector<Querier*> loadCSVMotionQueriers(const String& filename, int nqueriers, ManualQueryHandler* qh, std::tr1::function<Vector3()> gen_loc, float qradius) {
    std::vector<MotionAndBounds> data = loadCSVMotions(filename);
    std::vector<Querier*> results;

    for(unsigned int i = 0; i < (unsigned int)std::min((int)data.size(), nqueriers); i++) {
        int data_idx = rand() % data.size();
        results.push_back(
            new Querier(qh,
                MotionPath(gen_loc(), data[data_idx].motion, true),
                BoundingSphere(Vector3(0,0,0), data[data_idx].radius),
                qradius
            )
        );
    }

    return results;
}

} // namespace Simulation
} // namespace Prox
