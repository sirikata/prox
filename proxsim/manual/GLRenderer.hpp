// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_GLRENDERER_HPP_
#define _PROXSIM_MANUAL_GLRENDERER_HPP_

#include <proxsimcore/GLRendererBase.hpp>
#include "SimulatorQueryListener.hpp"

namespace Prox {
namespace Simulation {

class Simulator;

class GLRenderer : public GLRendererBase, public ManualQueryEventListener, public SimulatorQueryListener {
public:
    GLRenderer(Simulator* sim, ManualQueryHandler* handler, bool display = true);
    virtual ~GLRenderer();

    // QueryEventListener Interface
    virtual void queryHasEvents(ManualQuery* query);

    // SimulatorQueryListener Interface
    virtual void simulatorAddedQuery(Querier* query);
    virtual void simulatorRemovedQuery(Querier* query);

    // GLRenderer Interface
    virtual void display();

protected:
    GLRenderer();

    Simulator* mSimulator;
}; // class Renderer

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_GLRENDERER_HPP_
