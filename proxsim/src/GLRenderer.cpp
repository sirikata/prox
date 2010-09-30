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

static void color_palette_initialize() {
    if (color_palette_initialized) return;

    for(int i = 0; i < COLOR_PALETTE_COLORS; i++) {
        color_palette[i].r = (rand() % 255)/255.f;
        color_palette[i].g = (rand() % 255)/255.f;
        color_palette[i].b = (rand() % 255)/255.f;
    }
}

static GLRenderer* GLRenderer_sRenderer = NULL;


void glut_display() {
    GLRenderer_sRenderer->display();
}

void glut_reshape(int w, int h) {
    color_palette_initialize();
    GLRenderer_sRenderer->reshape(w, h);
}

void glut_keyboard(unsigned char key, int x, int y) {
    GLRenderer_sRenderer->keyboard(key, x, y);
}

void glut_timer(int val) {
    GLRenderer_sRenderer->timer();
}

GLRenderer::GLRenderer(Simulator* sim, QueryHandler* handler, bool display)
 : Renderer(sim),
   mDisplay(display),
   mWinWidth(0), mWinHeight(0),
   mMaxObservers(1),
   mDisplayMode(TimesSeen)
{
    mSimulator->addListener(this);
    handler->setAggregateListener(this);

    assert(GLRenderer_sRenderer == NULL);
    GLRenderer_sRenderer = this;

    int argc = 0;
    glutInit( &argc, NULL );

    if (display) {
        glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
        glutInitWindowSize( 512, 512 );

        glutCreateWindow( "Proximity Simulation" );

        glutDisplayFunc( glut_display );
        glutReshapeFunc( glut_reshape );
        glutKeyboardFunc( glut_keyboard );
    }
}

GLRenderer::~GLRenderer() {
    mSimulator->removeListener(this);
    GLRenderer_sRenderer = NULL;
}

void GLRenderer::run() {
    reshape(mWinWidth, mWinHeight);

    if (mDisplay) {
        glutTimerFunc(16, glut_timer, 0);
        glutMainLoop();
    }
    else {
        while(!mSimulator->finished())
            glut_timer(true);
    }
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

void GLRenderer::validateSeenObjects() {
#ifdef PROXDEBUG
    // Validate
    for (ObjectRefCountMap::iterator it = mSeenObjects.begin(); it != mSeenObjects.end(); it++) {
        assert(
            it->second == 0 ||
            mAggregateObjects.find(it->first) != mAggregateObjects.end() ||
            mSimulator->objectsFind(it->first) != mSimulator->objectsEnd()
        );
    }
#endif //PROXDEBUG
}

void GLRenderer::simulatorAddedObject(Object* obj, const MotionVector3& pos, const BoundingSphere& bounds) {
    // nothing, we draw directly from the iterators in the simulator
}

void GLRenderer::simulatorRemovedObject(Object* obj) {
    // nothing, we draw directly from the iterators in the simulator
}

void GLRenderer::simulatorAddedQuery(Query* query) {
    query->setEventListener(this);
}

void GLRenderer::simulatorRemovedQuery(Query* query) {
    query->setEventListener(NULL);
}

void GLRenderer::aggregateCreated(QueryHandlerType* handler, const ObjectIDType& objid) {
    assert( mAggregateObjects.find(objid) == mAggregateObjects.end() );
    mAggregateObjects[objid] = ObjectIDSet();
}

void GLRenderer::aggregateChildAdded(QueryHandlerType* handler, const ObjectIDType& objid, const ObjectIDType& child, const BoundingSphereType& bnds) {
    assert( mAggregateObjects.find(objid) != mAggregateObjects.end() );
    mAggregateObjects[objid].insert(child);

    if (mAggregateObjects.find(child) == mAggregateObjects.end())
        mAggregateBounds[objid] = bnds;
    else
        mAggregateBounds.erase(objid);
}

void GLRenderer::aggregateChildRemoved(QueryHandlerType* handler, const ObjectIDType& objid, const ObjectIDType& child, const BoundingSphereType& bnds) {
    assert( mAggregateObjects.find(objid) != mAggregateObjects.end() );
    mAggregateObjects[objid].erase(child);
}

void GLRenderer::aggregateBoundsUpdated(QueryHandlerType* handler, const ObjectIDType& objid, const BoundingSphereType& bnds) {
    if (mAggregateBounds.find(objid) != mAggregateBounds.end())
        mAggregateBounds[objid] = bnds;
}

void GLRenderer::aggregateDestroyed(QueryHandlerType* handler, const ObjectIDType& objid) {
    AggregateObjectMap::iterator it = mAggregateObjects.find(objid);
    assert(it != mAggregateObjects.end());
    mAggregateObjects.erase(it);

    mAggregateBounds.erase(objid);
}

void GLRenderer::aggregateObserved(QueryHandlerType* handler, const ObjectIDType& objid, uint32 nobservers) {
    assert( mAggregateObjects.find(objid) != mAggregateObjects.end() );
}

void GLRenderer::display() {
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glPointSize(4.f);

    switch(mDisplayMode) {
      case TimesSeen:
          {
              for(Simulator::ObjectIterator it = mSimulator->objectsBegin(); it != mSimulator->objectsEnd(); it++) {
                  Object* obj = it->second;
                  BoundingSphere bb = obj->worldBounds(mSimulator->time());

                  if (mSeenObjects.find(obj->id()) != mSeenObjects.end() && mSeenObjects[obj->id()] > 0) {
                      float col = 0.5f + 0.5f * ((float)mSeenObjects[obj->id()] / mMaxObservers);
                      glColor3f(col, col, col);
                  }
                  else
                      glColor3f(0.f, 0.f, 0.f);

                  //drawbb(bb);
                  glBegin(GL_POINTS);
                  glVertex3f(bb.center().x, bb.center().y, bb.center().z);
                  glEnd();
              }
          }
          break;
      case SmallestAggregates:
          {
              int idx = 0;
              for(AggregateBounds::iterator it = mAggregateBounds.begin(); it != mAggregateBounds.end(); it++) {
                  ObjectID agg = it->first;
                  BoundingSphere bb = it->second;

                  int col_idx = ObjectID::Hasher()(agg) % COLOR_PALETTE_COLORS;
                  glColor3f(color_palette[col_idx].r, color_palette[col_idx].g, color_palette[col_idx].b);

                  ObjectIDSet& children = mAggregateObjects[agg];
                  glBegin(GL_POINTS);
                  for(ObjectIDSet::iterator cit = children.begin(); cit != children.end(); cit++) {
                      Simulator::ObjectIterator oit = mSimulator->objectsFind(*cit);
                      if (oit == mSimulator->objectsEnd()) continue;
                      Object* obj = oit->second;
                      BoundingSphere cbb = obj->worldBounds(mSimulator->time());
                      glVertex3f(cbb.center().x, cbb.center().y, cbb.center().z);
                  }
                  glEnd();
              }
          }
          break;
    }

    glColor3f(1.f, 0.f, 0.f);
    for(Simulator::QueryIterator it = mSimulator->queriesBegin(); it != mSimulator->queriesEnd(); it++) {
        Query* query = *it;
        Vector3 center = query->position(mSimulator->time());
        glPushMatrix();
        glTranslatef(center.x, center.y, center.z);
        glutSolidSphere(1.f, 10, 10);
        glPopMatrix();
    }

    glutSwapBuffers();
}

void GLRenderer::reshape(int w, int h) {
    mWinWidth = w; mWinHeight = h;

    BoundingBox3 sim_bb = mSimulator->region();

    if (!mDisplay) return;

    glClearColor( .3, .3, .3, 1 );
    glClearDepth(1.0);

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();

    glOrtho( sim_bb.min().x, sim_bb.max().x, sim_bb.min().y, sim_bb.max().y, sim_bb.max().z, sim_bb.min().z );
    glViewport( 0, 0, mWinWidth, mWinHeight );

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    glEnable(GL_DEPTH_TEST);

    glutPostRedisplay();
}

void GLRenderer::timer() {
    //mSeenObjects.clear();
    mSimulator->tick();
    validateSeenObjects();
    if (mDisplay) {
        if (mSimulator->finished()) exit(0);
        glutTimerFunc(16, glut_timer, 0);
        glutPostRedisplay();
    }
}

void GLRenderer::keyboard(unsigned char key, int x, int y) {
    if (key == 27) // ESC
        exit(0);
    if (key == 'd')
        mDisplayMode = (DisplayMode) ( (mDisplayMode+1) % NumDisplayModes );
}

void GLRenderer::drawbb(const BoundingBox3& bb) {
    glBegin(GL_QUADS);

    glVertex3f ( bb.min().x,  bb.min().y,  bb.min().z);
    glVertex3f ( bb.max().x,  bb.min().y,  bb.min().z);
    glVertex3f ( bb.max().x,  bb.max().y,  bb.min().z);
    glVertex3f ( bb.min().x,  bb.max().y,  bb.min().z);

    glVertex3f ( bb.max().x,  bb.min().y,  bb.min().z);
    glVertex3f ( bb.max().x,  bb.min().y,  bb.max().z);
    glVertex3f ( bb.max().x,  bb.max().y,  bb.max().z);
    glVertex3f ( bb.max().x,  bb.max().y,  bb.min().z);

    glVertex3f ( bb.max().x,  bb.min().y,  bb.max().z);
    glVertex3f ( bb.min().x,  bb.min().y,  bb.max().z);
    glVertex3f ( bb.min().x,  bb.max().y,  bb.max().z);
    glVertex3f ( bb.max().x,  bb.max().y,  bb.max().z);

    glVertex3f ( bb.min().x,  bb.min().y,  bb.max().z);
    glVertex3f ( bb.min().x,  bb.min().y,  bb.min().z);
    glVertex3f ( bb.min().x,  bb.max().y,  bb.min().z);
    glVertex3f ( bb.min().x,  bb.max().y,  bb.max().z);

    glVertex3f ( bb.min().x,  bb.max().y,  bb.min().z);
    glVertex3f ( bb.max().x,  bb.max().y,  bb.min().z);
    glVertex3f ( bb.max().x,  bb.max().y,  bb.max().z);
    glVertex3f ( bb.min().x,  bb.max().y,  bb.max().z);

    glVertex3f ( bb.min().x,  bb.min().y,  bb.max().z);
    glVertex3f ( bb.max().x,  bb.min().y,  bb.max().z);
    glVertex3f ( bb.max().x,  bb.min().y,  bb.min().z);
    glVertex3f ( bb.min().x,  bb.min().y,  bb.min().z);

    glEnd();
}

void GLRenderer::drawbs(const BoundingSphere& bs) {
    Vector3 center = bs.center();
    float radius = bs.radius();
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    glutSolidSphere(radius, 10, 10);
    glPopMatrix();
}

} // namespace Simulation
} // namespace Prox
