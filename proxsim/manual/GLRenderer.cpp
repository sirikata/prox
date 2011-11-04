// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include <stdint.h>
#include "GLRenderer.hpp"
#include "Simulator.hpp"

#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif
#include <cassert>

using namespace Prox;

namespace Prox {
namespace Simulation {

GLRenderer::GLRenderer(Simulator* sim, ManualQueryHandler* handler, bool display)
 : GLRendererBase(sim, display),
   mSimulator(sim)
{
    mSimulator->addListener(this);
    handler->setAggregateListener(this);
}

GLRenderer::~GLRenderer() {
    mSimulator->removeListener(this);
}


void GLRenderer::queryHasEvents(ManualQuery* query) {
    std::deque<QueryEvent> evts;
    query->popEvents(evts);

    for(std::deque<QueryEvent>::iterator it = evts.begin(); it != evts.end(); it++) {
        for(QueryEvent::AdditionList::iterator add_it = it->additions().begin(); add_it != it->additions().end(); add_it++) {
            if (mSeenObjects.find(add_it->id()) == mSeenObjects.end())
                mSeenObjects[add_it->id()] = 0;
            mSeenObjects[add_it->id()]++;
            mMaxObservers = std::max(mMaxObservers, mSeenObjects[add_it->id()]);
        }

        for(QueryEvent::RemovalList::iterator rem_it = it->removals().begin(); rem_it != it->removals().end(); rem_it++) {
            assert(mSeenObjects.find(rem_it->id()) != mSeenObjects.end());
            mSeenObjects[rem_it->id()]--;
        }
    }
}

void GLRenderer::simulatorAddedQuery(Querier* query) {
    query->setEventListener(this);
}

void GLRenderer::simulatorRemovedQuery(Querier* query) {
    query->setEventListener(NULL);
}

void GLRenderer::display() {
    GLRendererBase::display();

    glColor3f(1.f, 0.f, 0.f);
    for(Simulator::QueryIterator it = mSimulator->queriesBegin(); it != mSimulator->queriesEnd(); it++) {
        Querier* query = *it;
        Vector3 center = query->position(mSimulator->time());
        glPushMatrix();
        glTranslatef(center.x, center.y, center.z);
        glutSolidSphere(1.f, 10, 10);
        glPopMatrix();
    }

    glutSwapBuffers();
}

} // namespace Simulation
} // namespace Prox
