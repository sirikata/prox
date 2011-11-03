// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include <stdint.h>
#include "GLRenderer.hpp"

#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif
#include <cassert>

using namespace Prox;

namespace Prox {
namespace Simulation {

GLRenderer::GLRenderer(SimulatorBase* sim, bool display)
 : GLRendererBase(sim, display),
   mSimulator(sim)
{
}

GLRenderer::~GLRenderer() {
}

void GLRenderer::display() {
    GLRendererBase::display();

    glutSwapBuffers();
}

} // namespace Simulation
} // namespace Prox
