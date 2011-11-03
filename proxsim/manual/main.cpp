// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include <proxsimcore/ObjectLocationServiceCache.hpp>
#include <proxsimcore/BoundingBox.hpp>

#include <prox/manual/Query.hpp>
#include <prox/manual/QueryHandler.hpp>

int main(int argc, char** argv) {
    using namespace Prox::Simulation;

    unsigned int seed = 0;
    srand(seed);

    BoundingBox3 random_region( Vector3(-100.f, -100.f, -100.f), Vector3(100.f, 100.f, 100.f) );

    return 0;
}
