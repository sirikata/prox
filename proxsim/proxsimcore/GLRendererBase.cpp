// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include <stdint.h>
#include "GLRendererBase.hpp"
#include "SimulatorBase.hpp"

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

static GLRendererBase* GLRendererBase_sRenderer = NULL;


void glut_display() {
    GLRendererBase_sRenderer->display();
}

void glut_reshape(int w, int h) {
    color_palette_initialize();
    GLRendererBase_sRenderer->reshape(w, h);
}

void glut_keyboard(unsigned char key, int x, int y) {
    GLRendererBase_sRenderer->keyboard(key, x, y);
}

void glut_timer(int val) {
    GLRendererBase_sRenderer->timer();
}

GLRendererBase::GLRendererBase(SimulatorBase* sim, bool display)
 : mSimulatorBase(sim),
   mDisplay(display),
   mWinWidth(0), mWinHeight(0),
   mMaxObservers(1),
   mDisplayMode(TimesSeen)
{

    assert(GLRendererBase_sRenderer == NULL);
    GLRendererBase_sRenderer = this;

    int argc = 0;
    if (display) {
        glutInit( &argc, NULL );
        glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
        glutInitWindowSize( 512, 512 );

        glutCreateWindow( "Proximity Simulation" );

        glutDisplayFunc( glut_display );
        glutReshapeFunc( glut_reshape );
        glutKeyboardFunc( glut_keyboard );
    }
}

GLRendererBase::~GLRendererBase() {
    GLRendererBase_sRenderer = NULL;
}

void GLRendererBase::run() {
    reshape(mWinWidth, mWinHeight);

    if (mDisplay) {
        glutTimerFunc(16, glut_timer, 0);
        glutMainLoop();
    }
    else {
        while(!mSimulatorBase->finished())
            glut_timer(true);
    }
}

void GLRendererBase::aggregateCreated(AggregatorType* handler, const ObjectIDType& objid) {
    Lock lck(mAggregateMutex);
    assert( mAggregateObjects.find(objid) == mAggregateObjects.end() );
    mAggregateObjects[objid] = ObjectIDSet();
}

void GLRendererBase::aggregateChildAdded(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
    const Vector3& bnds_center_offset, const float32 bnds_center_bounds_radius, const float32 bnds_max_object_size) {
    Lock lck(mAggregateMutex);
    assert( mAggregateObjects.find(objid) != mAggregateObjects.end() );
    mAggregateObjects[objid].insert(child);

    mAggregateBounds[objid] = BoundsInfo(bnds_center_offset, bnds_center_bounds_radius, bnds_max_object_size);
}

void GLRendererBase::aggregateChildRemoved(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child,
    const Vector3& bnds_center_offset, const float32 bnds_center_bounds_radius, const float32 bnds_max_object_size) {
    Lock lck(mAggregateMutex);
    assert( mAggregateObjects.find(objid) != mAggregateObjects.end() );
    mAggregateObjects[objid].erase(child);
}

void GLRendererBase::aggregateBoundsUpdated(AggregatorType* handler, const ObjectIDType& objid,
    const Vector3& bnds_center_offset, const float32 bnds_center_bounds_radius, const float32 bnds_max_object_size) {
    Lock lck(mAggregateMutex);
    if (mAggregateBounds.find(objid) != mAggregateBounds.end())
        mAggregateBounds[objid] = BoundsInfo(bnds_center_offset, bnds_center_bounds_radius, bnds_max_object_size);
}

void GLRendererBase::aggregateDestroyed(AggregatorType* handler, const ObjectIDType& objid) {
    Lock lck(mAggregateMutex);
    AggregateObjectMap::iterator it = mAggregateObjects.find(objid);
    assert(it != mAggregateObjects.end());
    mAggregateObjects.erase(it);

    mAggregateBounds.erase(objid);
}

void GLRendererBase::aggregateObserved(AggregatorType* handler, const ObjectIDType& objid, uint32 nobservers) {
    Lock lck(mAggregateMutex);
    assert( mAggregateObjects.find(objid) != mAggregateObjects.end() );
}

void GLRendererBase::validateSeenObjects() {
#ifdef PROXDEBUG
    Lock lck(mAggregateMutex);
    // Validate
    for (ObjectRefCountMap::iterator it = mSeenObjects.begin(); it != mSeenObjects.end(); it++) {
        assert(
            it->second == 0 ||
            mAggregateObjects.find(it->first) != mAggregateObjects.end() ||
            mSimulatorBase->objectsFind(it->first) != mSimulatorBase->objectsEnd()
        );
    }
#endif //PROXDEBUG
}

void GLRendererBase::display() {
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glPointSize(4.f);

    switch(mDisplayMode) {
      case TimesSeen:
          {
              for(SimulatorBase::ObjectIterator it = mSimulatorBase->objectsBegin(); it != mSimulatorBase->objectsEnd(); it++) {
                  Object* obj = it->second;
                  BoundingSphere bb = obj->worldBounds(mSimulatorBase->time());

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
              Lock lck(mAggregateMutex);
              int idx = 0;
              for(AggregateBounds::iterator it = mAggregateBounds.begin(); it != mAggregateBounds.end(); it++) {
                  ObjectID agg = it->first;
                  BoundingSphere bb(it->second.centerOffset, it->second.centerBoundsRadius + it->second.maxObjectSize);

                  int col_idx = ObjectID::Hasher()(agg) % COLOR_PALETTE_COLORS;
                  glColor3f(color_palette[col_idx].r, color_palette[col_idx].g, color_palette[col_idx].b);

                  ObjectIDSet& children = mAggregateObjects[agg];
                  glBegin(GL_POINTS);
                  for(ObjectIDSet::iterator cit = children.begin(); cit != children.end(); cit++) {
                      SimulatorBase::ObjectIterator oit = mSimulatorBase->objectsFind(*cit);
                      if (oit == mSimulatorBase->objectsEnd()) continue;
                      Object* obj = oit->second;
                      BoundingSphere cbb = obj->worldBounds(mSimulatorBase->time());
                      glVertex3f(cbb.center().x, cbb.center().y, cbb.center().z);
                  }
                  glEnd();
              }
          }
          break;
      case SeenWithAggregates:
          {
              for(ObjectRefCountMap::const_iterator it = mSeenObjects.begin(); it != mSeenObjects.end(); it++) {
                  const ObjectID& objid = it->first;
                  uint32 num_seen = it->second;
                  if (num_seen == 0) continue;

                  BoundingSphere bb;
                  SimulatorBase::ObjectIterator oit = mSimulatorBase->objectsFind(objid);
                  if (oit != mSimulatorBase->objectsEnd()) {
                      Object* obj = oit->second;
                      bb = obj->worldBounds(mSimulatorBase->time());
                  }
                  else {
                      AggregateBounds::const_iterator agg_bounds_it = mAggregateBounds.find(objid);
                      if (agg_bounds_it == mAggregateBounds.end()) continue;
                      bb = BoundingSphere(agg_bounds_it->second.centerOffset, agg_bounds_it->second.centerBoundsRadius + agg_bounds_it->second.maxObjectSize);
                  }

                  float col = 0.5f + 0.5f * ((float)num_seen / mMaxObservers);
                  glColor3f(col, col, col);

                  glBegin(GL_POINTS);
                  glVertex3f(bb.center().x, bb.center().y, bb.center().z);
                  glEnd();
              }
          }
          break;
      case NumDisplayModes:
        assert(false);
        break;
    }
}

void GLRendererBase::reshape(int w, int h) {
    mWinWidth = w; mWinHeight = h;

    BoundingBox3 sim_bb = mSimulatorBase->region();

    if (!mDisplay) return;

    glClearColor( .3, .3, .3, 1 );
    glClearDepth(1.0);

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();

    float zsize = 2 * std::max(fabs(sim_bb.max().z), fabs(sim_bb.min().z)) + 1;
    glOrtho( sim_bb.min().x, sim_bb.max().x, sim_bb.min().y, sim_bb.max().y, sim_bb.max().z + zsize, sim_bb.min().z - zsize);
    glViewport( 0, 0, mWinWidth, mWinHeight );

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    glEnable(GL_DEPTH_TEST);

    glutPostRedisplay();
}

void GLRendererBase::timer() {
    //mSeenObjects.clear();
    mSimulatorBase->tick();
    validateSeenObjects();
    if (mDisplay) {
        if (mSimulatorBase->finished()) exit(0);
        glutTimerFunc(16, glut_timer, 0);
        glutPostRedisplay();
    }
}

void GLRendererBase::keyboard(unsigned char key, int x, int y) {
    if (key == 27) // ESC
        exit(0);
    if (key == 'd')
        mDisplayMode = (DisplayMode) ( (mDisplayMode+1) % NumDisplayModes );
    if (key == 'n')
        mSimulatorBase->printNodes();
}

void GLRendererBase::drawbb(const BoundingBox3& bb) {
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

void GLRendererBase::drawbs(const BoundingSphere& bs) {
    Vector3 center = bs.center();
    float radius = bs.radius();
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    glutSolidSphere(radius, 10, 10);
    glPopMatrix();
}

} // namespace Simulation
} // namespace Prox
