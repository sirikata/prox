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
//#include "GLRenderer.hpp"
//#include <prox/BruteForceQueryHandler.hpp>
//#include <prox/RTreeQueryHandler.hpp>
//#include "ObjectLocationServiceCache.hpp"

#include <iostream>
#include <string>
#include <stdint.h>


int main(int argc, char** argv) {
    using namespace Prox::Simulation;

    // Parse arguments
    std::string HANDLER_ARG("--handler=");
    std::string handler_type = "brute";
    for(int argi = 0; argi < argc; argi++) {
        std::string arg(argv[argi]);
        if (arg.find(HANDLER_ARG) != std::string::npos)
            handler_type = arg.substr(HANDLER_ARG.size());
    }

    // Setup query handler
    QueryHandler* handler = NULL;
    if (handler_type == "rtree")
        handler = new Prox::RTreeQueryHandler<>(10);
    else
        handler = new Prox::BruteForceQueryHandler<>();

    Simulator* simulator = new Simulator(handler);
    Renderer* renderer = new GLRenderer(simulator);

    simulator->initialize(Time::null(), BoundingBox3( Vector3(-100.f, -100.f, -100.f), Vector3(100.f, 100.f, 100.f) ), 10000, 50);

    renderer->run();

    delete renderer;
    delete simulator;

    return 0;
}
