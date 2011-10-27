/*  proxsim
 *  main.cpp
 *
 *  Copyright (c) 2009, Ewen Cheslack-Postava
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of libprox nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Simulator.hpp"
#include "GLRenderer.hpp"
#include <prox/BruteForceQueryHandler.hpp>
#include <prox/RTreeAngleQueryHandler.hpp>
#include <prox/RTreeDistanceQueryHandler.hpp>
#include <prox/RTreeCutQueryHandler.hpp>
#include <prox/RebuildingQueryHandler.hpp>
#include "ObjectLocationServiceCache.hpp"

#include <iostream>
#include <string>
#include <stdint.h>

#include <boost/lexical_cast.hpp>

static bool convert_bool(const std::string& arg) {
    return (arg == "on" || arg == "true" || arg == "yes");
}

template<typename T>
void convert_range(const std::string& arg, T* rmin, T* rmax) {
    if (arg.find("(") != std::string::npos) {
        // A real range of the format "(x,y)"
        int lparen = arg.find("(");
        int comma = arg.find(",");
        int rparen = arg.find(")");
        assert((std::size_t)lparen != std::string::npos);
        assert((std::size_t)comma != std::string::npos);
        assert((std::size_t)rparen != std::string::npos);
        std::string first = arg.substr( lparen+1, (comma-(lparen+1)) );
        std::string second = arg.substr( comma+1, (rparen-(comma+1)) );
        *rmin = boost::lexical_cast<T>(first);
        *rmax = boost::lexical_cast<T>(second);
    }
    else {
        // Just a single value
        T val = boost::lexical_cast<T>(arg);
        *rmin = val;
        *rmax = val;
    }
}

int main(int argc, char** argv) {
    using namespace Prox::Simulation;

    // Parse arguments
    std::string HANDLER_ARG("--handler=");
    std::string DISPLAY_ARG("--display=");
    std::string BRANCH_ARG("--branch=");
    std::string NOBJECTS_ARG("--nobjects=");
    std::string STATIC_OBJECTS_ARG("--static-objects=");
    std::string MOVING_FRAC_ARG("--moving-frac=");
    std::string NQUERIES_ARG("--nqueries=");
    std::string STATIC_QUERIES_ARG("--static-queries=");
    std::string QUERY_ANGLE_ARG("--query-angle=");
    std::string QUERY_DISTANCE_ARG("--query-distance=");
    std::string QUERY_RESULTS_ARG("--query-results=");
    std::string DURATION_ARG("--duration=");
    std::string ITERATIONS_ARG("--iterations=");
    std::string REALTIME_ARG("--realtime=");
    std::string TIMESTEP_ARG("--timestep="); // in milliseconds
    std::string TRACK_CHECKS_ARG("--track-checks=");
    std::string RESTRUCTURE_ARG("--restructure=");
    std::string REPORT_HEALTH_ARG("--report-health=");
    std::string REPORT_HEALTH_FREQUENCY_ARG("--report-health-frequency=");
    std::string REPORT_COST_ARG("--report-cost=");
    std::string REPORT_RATE_ARG("--report-rate=");
    std::string REPORT_RESTRUCTURES_ARG("--report-restructures=");
    std::string REPORT_QUERY_STATS_ARG("--report-query-stats=");
    std::string CHURN_RATE_ARG("--churn-rate=");
    std::string FORCE_REBUILD_ARG("--force-rebuild=");
    std::string FORCE_INITIAL_REBUILD_ARG("--force-initial-rebuild=");
    std::string CSV_ARG("--csv=");
    std::string CSV_MOTION_ARG("--csvmotion=");
    std::string SEED_ARG("--seed=");
    std::string handler_type = "brute";

    int seed = 0;
    bool display = false;
    int branching = 16;
    int nobjects = 10000;
    int nqueries = 50;
    bool static_queries = false;
    float query_angle_min = (SolidAngle::Max / 1000.f).asFloat(), query_angle_max = (SolidAngle::Max / 1000.f).asFloat();
    float query_max_distance = Prox::DefaultSimulationTraits::InfiniteRadius;
    unsigned int query_max_results = Prox::DefaultSimulationTraits::InfiniteResults;
    int duration = 0; // seconds
    int iterations = 0; // iterations before termination
    bool realtime = true; // realtime or simulated time steps
    int timestep = 50;
    bool track_checks = false;
    bool restructure = false;
    bool report_health = false;
    int report_health_frequency = 1;
    bool report_cost = false;
    bool report_rate = false;
    bool report_restructures = false;
    bool report_query_stats = false;
    int churn_rate = 0;
    bool force_rebuild = false;
    bool force_initial_rebuild = false;
    float moving_frac = 1.0f;
    std::string csvfile = "";
    std::string csvmotionfile = "";

    for(int argi = 0; argi < argc; argi++) {
        std::string arg(argv[argi]);
        if (arg.find(HANDLER_ARG) != std::string::npos)
            handler_type = arg.substr(HANDLER_ARG.size());
        else if (arg.find(DISPLAY_ARG) != std::string::npos) {
            std::string display_arg = arg.substr(DISPLAY_ARG.size());
            display = convert_bool(display_arg);
        }
        else if (arg.find(BRANCH_ARG) != std::string::npos) {
            std::string branch_arg = arg.substr(BRANCH_ARG.size());
            branching = boost::lexical_cast<int>(branch_arg);
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
        else if (arg.find(NQUERIES_ARG) != std::string::npos) {
            std::string nqueries_arg = arg.substr(NQUERIES_ARG.size());
            nqueries = boost::lexical_cast<int>(nqueries_arg);
        }
        else if (arg.find(STATIC_QUERIES_ARG) != std::string::npos) {
            std::string static_arg = arg.substr(STATIC_QUERIES_ARG.size());
            static_queries = convert_bool(static_arg);
        }
        else if (arg.find(QUERY_ANGLE_ARG) != std::string::npos) {
            std::string qangle_arg = arg.substr(QUERY_ANGLE_ARG.size());
            convert_range(qangle_arg, &query_angle_min, &query_angle_max);
        }
        else if (arg.find(QUERY_DISTANCE_ARG) != std::string::npos) {
            std::string qdistance_arg = arg.substr(QUERY_DISTANCE_ARG.size());
            query_max_distance = boost::lexical_cast<float>(qdistance_arg);
        }
        else if (arg.find(QUERY_RESULTS_ARG) != std::string::npos) {
            std::string qresults_arg = arg.substr(QUERY_RESULTS_ARG.size());
            query_max_results = boost::lexical_cast<unsigned int>(qresults_arg);
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
        else if (arg.find(TRACK_CHECKS_ARG) != std::string::npos) {
            std::string track_checks_arg = arg.substr(TRACK_CHECKS_ARG.size());
            track_checks = convert_bool(track_checks_arg);
        }
        else if (arg.find(RESTRUCTURE_ARG) != std::string::npos) {
            std::string restructure_arg = arg.substr(RESTRUCTURE_ARG.size());
            restructure = convert_bool(restructure_arg);
        }
        else if (arg.find(REPORT_HEALTH_ARG) != std::string::npos) {
            std::string report_health_arg = arg.substr(REPORT_HEALTH_ARG.size());
            report_health = convert_bool(report_health_arg);
        }
        else if (arg.find(REPORT_HEALTH_FREQUENCY_ARG) != std::string::npos) {
            std::string report_health_frequency_arg = arg.substr(REPORT_HEALTH_FREQUENCY_ARG.size());
            report_health_frequency = boost::lexical_cast<int>(report_health_frequency_arg);
        }
        else if (arg.find(REPORT_COST_ARG) != std::string::npos) {
            std::string report_cost_arg = arg.substr(REPORT_COST_ARG.size());
            report_cost = convert_bool(report_cost_arg);
        }
        else if (arg.find(REPORT_RATE_ARG) != std::string::npos) {
            std::string report_rate_arg = arg.substr(REPORT_RATE_ARG.size());
            report_rate = convert_bool(report_rate_arg);
        }
        else if (arg.find(REPORT_RESTRUCTURES_ARG) != std::string::npos) {
            std::string report_restructures_arg = arg.substr(REPORT_RESTRUCTURES_ARG.size());
            report_restructures = convert_bool(report_restructures_arg);
        }
        else if (arg.find(REPORT_QUERY_STATS_ARG) != std::string::npos) {
            std::string report_query_stats_arg = arg.substr(REPORT_QUERY_STATS_ARG.size());
            report_query_stats = convert_bool(report_query_stats_arg);
        }
        else if (arg.find(CHURN_RATE_ARG) != std::string::npos) {
            std::string churn_rate_arg = arg.substr(CHURN_RATE_ARG.size());
            churn_rate = convert_bool(churn_rate_arg);
        }
        else if (arg.find(FORCE_REBUILD_ARG) != std::string::npos) {
            std::string force_rebuild_arg = arg.substr(FORCE_REBUILD_ARG.size());
            force_rebuild = convert_bool(force_rebuild_arg);
        }
        else if (arg.find(FORCE_INITIAL_REBUILD_ARG) != std::string::npos) {
            std::string force_initial_rebuild_arg = arg.substr(FORCE_INITIAL_REBUILD_ARG.size());
            force_initial_rebuild = convert_bool(force_initial_rebuild_arg);
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

    // Setup query handler
    QueryHandler* handler = NULL;
    if (handler_type == "rtree") {
        handler = new Prox::RebuildingQueryHandler<>(
            Prox::RTreeAngleQueryHandler<>::Constructor(branching), 10
        );
    }
    else if (handler_type == "rtreedist") {
        handler = new Prox::RebuildingQueryHandler<>(
            Prox::RTreeDistanceQueryHandler<>::Constructor(branching), 10
        );
        // In case they didn't reduce it, force a lower default
        if (query_max_distance == Prox::DefaultSimulationTraits::InfiniteRadius)
            query_max_distance = 20; // Reasonable match for 200x200x200 region
    }
    else if (handler_type == "rtreecut") {
        handler = new Prox::RebuildingQueryHandler<>(
            Prox::RTreeCutQueryHandler<>::Constructor(branching, false), 10
        );
    }
    else if (handler_type == "rtreecutagg") {
        handler = new Prox::RebuildingQueryHandler<>(
            Prox::RTreeCutQueryHandler<>::Constructor(branching, true), 10
        );
    }
    else {
        handler = new Prox::RebuildingQueryHandler<>(
            Prox::BruteForceQueryHandler<>::Constructor(), 10
        );
    }

    Simulator* simulator = new Simulator(handler, duration, Duration::milliseconds((unsigned int)timestep), iterations, realtime);
    Renderer* renderer = new GLRenderer(simulator, handler, display);

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

    simulator->initialize(churn_rate, SolidAngle(query_angle_min), SolidAngle(query_angle_max), query_max_distance, query_max_results);

    if (!csvmotionfile.empty() && !static_queries)
        simulator->createCSVQueries(nqueries, csvmotionfile);
    else
        simulator->createRandomQueries(nqueries, static_queries);

    simulator->run();

    // Optional logging, triggers
    handler->trackChecks(track_checks);
    handler->shouldRestructure(restructure);
    handler->reportHealth(report_health);
    handler->reportHealthFrequency(report_health_frequency);
    handler->reportCost(report_cost);
    handler->setReportRestructures(report_restructures);
    handler->reportQueryStats(report_query_stats);
    simulator->printRate(report_rate);
    simulator->forceRebuild(force_rebuild);
    simulator->forceInitialRebuild(force_initial_rebuild);

    renderer->run();
    simulator->shutdown();

    delete renderer;
    delete simulator;

    return 0;
}
