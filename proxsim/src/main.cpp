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
    return (arg == "on" || arg == "true");
}

int main(int argc, char** argv) {
    using namespace Prox::Simulation;

    // Parse arguments
    std::string HANDLER_ARG("--handler=");
    std::string DISPLAY_ARG("--display=");
    std::string BRANCH_ARG("--branch=");
    std::string NOBJECTS_ARG("--nobjects=");
    std::string NQUERIES_ARG("--nqueries=");
    std::string TRACK_CHECKS_ARG("--track-checks=");
    std::string handler_type = "brute";
    bool display = true;
    int branching = 16;
    int nobjects = 10000;
    int nqueries = 50;
    bool track_checks = false;
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
        else if (arg.find(NQUERIES_ARG) != std::string::npos) {
            std::string nqueries_arg = arg.substr(NQUERIES_ARG.size());
            nqueries = boost::lexical_cast<int>(nqueries_arg);
        }
        else if (arg.find(TRACK_CHECKS_ARG) != std::string::npos) {
            std::string track_checks_arg = arg.substr(TRACK_CHECKS_ARG.size());
            track_checks = convert_bool(track_checks_arg);
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

    Simulator* simulator = new Simulator(handler);
    Renderer* renderer = new GLRenderer(simulator, handler, display);

    simulator->initialize(Time::null(), BoundingBox3( Vector3(-100.f, -100.f, -100.f), Vector3(100.f, 100.f, 100.f) ), nobjects, nqueries, 100);

    // Optional logging
    handler->trackChecks(track_checks);

    renderer->run();

    delete renderer;
    delete simulator;

    return 0;
}
