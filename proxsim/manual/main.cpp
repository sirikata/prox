// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include <proxsimcore/ObjectLocationServiceCache.hpp>
#include <proxsimcore/BoundingBox.hpp>

#include <prox/manual/Query.hpp>
#include <prox/manual/QueryHandler.hpp>

#include "Simulator.hpp"
#include "GLRenderer.hpp"

#include <proxsimcore/Convert.hpp>

int main(int argc, char** argv) {
    using namespace Prox::Simulation;

    std::string SEED_ARG("--seed=");
    std::string DISPLAY_ARG("--display=");

    std::string DURATION_ARG("--duration=");
    std::string ITERATIONS_ARG("--iterations=");
    std::string REALTIME_ARG("--realtime=");
    std::string TIMESTEP_ARG("--timestep="); // in milliseconds

    std::string NOBJECTS_ARG("--nobjects=");
    std::string STATIC_OBJECTS_ARG("--static-objects=");
    std::string MOVING_FRAC_ARG("--moving-frac=");
    std::string CSV_ARG("--csv=");
    std::string CSV_MOTION_ARG("--csvmotion=");

    unsigned int seed = 0;
    bool display = false;

    int duration = 0; // seconds
    int iterations = 0; // iterations before termination
    bool realtime = true; // realtime or simulated time steps
    int timestep = 50;

    int nobjects = 10000;
    float moving_frac = 1.0f;
    std::string csvfile = "";
    std::string csvmotionfile = "";

    for(int argi = 0; argi < argc; argi++) {
        std::string arg(argv[argi]);
        if (arg.find(SEED_ARG) != std::string::npos) {
            std::string seed_arg = arg.substr(SEED_ARG.size());
            seed = boost::lexical_cast<int>(seed_arg);
        }
        else if (arg.find(DISPLAY_ARG) != std::string::npos) {
            std::string display_arg = arg.substr(DISPLAY_ARG.size());
            display = convert_bool(display_arg);
        }

        else if (arg.find(DURATION_ARG) != std::string::npos) {
            std::string duration_arg = arg.substr(DURATION_ARG.size());
            duration = boost::lexical_cast<int>(duration_arg);
        }
        else if (arg.find(ITERATIONS_ARG) != std::string::npos) {
            std::string iterations_arg = arg.substr(ITERATIONS_ARG.size());
            iterations = boost::lexical_cast<int>(iterations_arg);
        }
        else if (arg.find(REALTIME_ARG) != std::string::npos) {
            std::string realtime_arg = arg.substr(REALTIME_ARG.size());
            realtime = convert_bool(realtime_arg);
        }
        else if (arg.find(TIMESTEP_ARG) != std::string::npos) {
            std::string timestep_arg = arg.substr(TIMESTEP_ARG.size());
            timestep = boost::lexical_cast<int>(timestep_arg);
        }

        else if (arg.find(NOBJECTS_ARG) != std::string::npos) {
            std::string nobjects_arg = arg.substr(NOBJECTS_ARG.size());
            nobjects = boost::lexical_cast<int>(nobjects_arg);
        }
        else if (arg.find(STATIC_OBJECTS_ARG) != std::string::npos) {
            std::string static_arg = arg.substr(STATIC_OBJECTS_ARG.size());
            bool static_objects = convert_bool(static_arg);
            if (static_objects) moving_frac = 0.0f;
        }
        else if (arg.find(MOVING_FRAC_ARG) != std::string::npos) {
            std::string moving_arg = arg.substr(MOVING_FRAC_ARG.size());
            moving_frac = boost::lexical_cast<float>(moving_arg);
        }
        else if (arg.find(CSV_ARG) != std::string::npos) {
            csvfile = arg.substr(CSV_ARG.size());
        }
        else if (arg.find(SEED_ARG) != std::string::npos) {
            std::string seed_arg = arg.substr(SEED_ARG.size());
            seed = boost::lexical_cast<int>(seed_arg);
        }
    }

    srand(seed);

    Simulator* simulator = new Simulator(duration, Duration::milliseconds((unsigned int)timestep), iterations, realtime);
    GLRenderer* renderer = new GLRenderer(simulator, display);

    BoundingBox3 random_region( Vector3(-100.f, -100.f, -100.f), Vector3(100.f, 100.f, 100.f) );

    int nobjects_moving = nobjects * moving_frac;
    int nobjects_static = nobjects - nobjects_moving;

    // There are various combinations of sources of objects we might need to get
    // to the target number with the right mix. When we need random objects and
    // we've loaded some from another source, we need to make sure we get the
    // region to generate them over correct.

    bool got_static = false;
    bool got_moving = false;

    // First, get objects from csv files.
    if (!csvfile.empty()) {
        simulator->createStaticCSVObjects(csvfile, nobjects_static);
        got_static = true;
    }
    // note: this should be second so that
    if (!csvmotionfile.empty()) {
        assert(!csvfile.empty()); // FIXME we'd like to support this, need to
                                  // figure out bounding box issues for
                                  // generating starting positions
        simulator->createMotionCSVObjects(csvmotionfile, nobjects_moving);
        got_moving = true;
    }

    // Next, take care of leftovers with random objects. Note that we use the
    // existing bounds if some other objects were already loaded.
    if (!got_static && !got_moving)
        simulator->createRandomObjects(random_region, nobjects, moving_frac);
    else if (!got_static && got_moving)
        simulator->createRandomObjects(simulator->region(), nobjects_static, 0.0);
    else if (got_static && !got_moving)
        simulator->createRandomObjects(simulator->region(), nobjects_moving, 1.0);
    // else we don't need any random objects

    // Sometimes we're not perfect, but let's aim for 99% of the target objects.
    assert(simulator->allObjectsSize() >= .99f * nobjects);

    simulator->initialize(/*churn_rate*/0);

    simulator->run();

    renderer->run();
    simulator->shutdown();

    delete renderer;
    delete simulator;


    return 0;
}
