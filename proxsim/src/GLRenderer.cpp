/*  proxsim
 *  GLRenderer.cpp
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

struct GLColor {
    float r, g, b;
};
#define COLOR_PALETTE_COLORS 1024
static GLColor color_palette[COLOR_PALETTE_COLORS];
static bool color_palette_initialized = false;

GLRenderer::GLRenderer(Simulator* sim, QueryHandler* handler, bool display)
 : GLRendererBase(sim, display),
   mSimulator(sim),
   mHandler(handler)
{
    mSimulator->addListener(this);
    handler->setAggregateListener(this);
}

GLRenderer::~GLRenderer() {
    mSimulator->removeListener(this);
}

void GLRenderer::queryHasEvents(Query* query) {
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

void GLRenderer::keyboard(unsigned char key, int x, int y) {
    GLRendererBase::keyboard(key, x, y);
    if (key == 'r')
        mHandler->rebuild();
}

} // namespace Simulation
} // namespace Prox
