/*  proxsim
 *  GLRenderer.hpp
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

#ifndef _PROXSIM_GLRENDERER_HPP_
#define _PROXSIM_GLRENDERER_HPP_

#include "SimulationTypes.hpp"
#include "Renderer.hpp"
#include "BoundingBox.hpp"
#include "SimulatorListener.hpp"
#include "Timer.hpp"

namespace Prox {
namespace Simulation {

class GLRenderer : public Renderer, public QueryEventListener, public SimulatorListener, public AggregateListener {
public:
    GLRenderer(Simulator* sim, QueryHandler* handler);
    virtual ~GLRenderer();

    // Renderer Interface
    virtual void run();

    // QueryEventListener Interface
    virtual void queryHasEvents(Query* query);

    // SimulatorListener Interface
    virtual void simulatorAddedObject(Object* obj, const MotionVector3& pos, const BoundingSphere& bounds);
    virtual void simulatorRemovedObject(Object* obj);
    virtual void simulatorAddedQuery(Query* query);
    virtual void simulatorRemovedQuery(Query* query);

    // GLRenderer Interface
    void display();
    void reshape(int w, int h);
    void timer();
    void keyboard(unsigned char key, int x, int y);

protected:
    // AggregateListener Interface
    virtual void aggregateCreated(const ObjectIDType& objid);
    virtual void aggregateChildAdded(const ObjectIDType& objid, const ObjectIDType& child);
    virtual void aggregateChildRemoved(const ObjectIDType& objid, const ObjectIDType& child);
    virtual void aggregateDestroyed(const ObjectIDType& objid);
    virtual void aggregateObserved(const ObjectIDType& objid, uint32 nobservers);

    GLRenderer();

    void drawbb(const BoundingBox3& bb);
    void drawbs(const BoundingSphere& bs);

    Time mTime;
    std::tr1::unordered_map<ObjectID, uint32, ObjectID::Hasher> mSeenObjects;
    Timer mTimer;
    int mWinWidth, mWinHeight;

    // Gets bounds of an object or aggregate object
    BoundingSphere getBounds(const ObjectID& obj);

    typedef std::tr1::unordered_set<ObjectID, ObjectID::Hasher> ObjectIDSet;
    typedef std::tr1::unordered_map<ObjectID, ObjectIDSet, ObjectID::Hasher> AggregateObjectMap;
    AggregateObjectMap mAggregateObjects;
}; // class Renderer

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_GLRENDERER_HPP_
