// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_GLRENDERER_HPP_
#define _PROXSIM_MANUAL_GLRENDERER_HPP_

#include <proxsimcore/GLRendererBase.hpp>

namespace Prox {
namespace Simulation {

class Simulator;

class GLRenderer : public GLRendererBase {
public:
    GLRenderer(SimulatorBase* sim, bool display = true);
    virtual ~GLRenderer();

    // GLRenderer Interface
    virtual void display();

protected:
    GLRenderer();

    SimulatorBase* mSimulator;
}; // class Renderer

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_GLRENDERER_HPP_
