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
#include <prox/RTreeQueryHandler.hpp>
#include <prox/RTreeCutQueryHandler.hpp>
#include "ObjectLocationServiceCache.hpp"

#include <iostream>
#include <string>
#include <stdint.h>

#include <boost/lexical_cast.hpp>

static bool convert_bool(const std::string& arg) {
    return (arg == "on" || arg == "true" || arg == "yes");
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
    std::string DURATION_ARG("--duration=");
    std::string ITERATIONS_ARG("--iterations=");
    std::string REALTIME_ARG("--realtime=");
    std::string TIMESTEP_ARG("--timestep="); // in milliseconds
    std::string TRACK_CHECKS_ARG("--track-checks=");
    std::string RESTRUCTURE_ARG("--restructure=");
    std::string REPORT_HEALTH_ARG("--report-health=");
    std::string REPORT_HEALTH_FREQUENCY_ARG("--report-health-frequency=");
    std::string REPORT_RATE_ARG("--report-rate=");
    std::string REPORT_RESTRUCTURES_ARG("--report-restructures=");
    std::string CHURN_RATE_ARG("--churn-rate=");
    std::string handler_type = "brute";
    bool display = false;
    int branching = 16;
    int nobjects = 10000;
    int nqueries = 50;
    bool static_queries = false;
    int duration = 0; // seconds
    int iterations = 0; // iterations before termination
    bool realtime = true; // realtime or simulated time steps
    int timestep = 50;
    bool track_checks = false;
    bool restructure = false;
    bool report_health = false;
    int report_health_frequency = 1;
    bool report_rate = false;
    bool report_restructures = false;
    int churn_rate = 0;
    float moving_frac = 1.0f;
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
        else if (arg.find(REPORT_RATE_ARG) != std::string::npos) {
            std::string report_rate_arg = arg.substr(REPORT_RATE_ARG.size());
            report_rate = convert_bool(report_rate_arg);
        }
        else if (arg.find(REPORT_RESTRUCTURES_ARG) != std::string::npos) {
            std::string report_restructures_arg = arg.substr(REPORT_RESTRUCTURES_ARG.size());
            report_restructures = convert_bool(report_restructures_arg);
        }
        else if (arg.find(CHURN_RATE_ARG) != std::string::npos) {
            std::string churn_rate_arg = arg.substr(CHURN_RATE_ARG.size());
            churn_rate = convert_bool(churn_rate_arg);
        }
    }

    // Setup query handler
    QueryHandler* handler = NULL;
    if (handler_type == "rtree")
        handler = new Prox::RTreeQueryHandler<>(branching);
    else if (handler_type == "rtreecut")
        handler = new Prox::RTreeCutQueryHandler<>(branching, false);
    else if (handler_type == "rtreecutagg")
        handler = new Prox::RTreeCutQueryHandler<>(branching, true);
    else
        handler = new Prox::BruteForceQueryHandler<>();

    Simulator* simulator = new Simulator(handler, duration, Duration::milliseconds((unsigned int)timestep), iterations, realtime);
    Renderer* renderer = new GLRenderer(simulator, handler, display);

    simulator->initialize(BoundingBox3( Vector3(-100.f, -100.f, -100.f), Vector3(100.f, 100.f, 100.f) ), nobjects, moving_frac, nqueries, static_queries, churn_rate);

    // Optional logging, triggers
    handler->trackChecks(track_checks);
    handler->shouldRestructure(restructure);
    handler->reportHealth(report_health);
    handler->reportHealthFrequency(report_health_frequency);
    handler->reportRestructures(report_restructures);
    simulator->printRate(report_rate);

    renderer->run();

    simulator->shutdown();

    delete renderer;
    delete simulator;

    return 0;
}
