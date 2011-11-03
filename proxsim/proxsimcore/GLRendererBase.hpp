// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIMCORE_GLRENDERERBASE_HPP_
#define _PROXSIMCORE_GLRENDERERBASE_HPP_

#include <proxsimcore/SimulationTypes.hpp>
#include <proxsimcore/BoundingBox.hpp>
#include <proxsimcore/Timer.hpp>

namespace Prox {
namespace Simulation {

class SimulatorBase;

class GLRendererBase : public AggregateListener {
public:
    GLRendererBase(SimulatorBase* sim, bool display = true);
    virtual ~GLRendererBase();

    // Renderer Interface
    virtual void run();

    // GLRendererBase Interface
    // Implementations are repsonible for calling glSwapBuffers!
    virtual void display();
    virtual void reshape(int w, int h);
    virtual void timer();
    virtual void keyboard(unsigned char key, int x, int y);

protected:
    // AggregateListener Interface
    virtual void aggregateCreated(AggregatorType* handler, const ObjectIDType& objid);
    virtual void aggregateChildAdded(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child, const BoundingSphereType& bnds);
    virtual void aggregateChildRemoved(AggregatorType* handler, const ObjectIDType& objid, const ObjectIDType& child, const BoundingSphereType& bnds);
    virtual void aggregateBoundsUpdated(AggregatorType* handler, const ObjectIDType& objid, const BoundingSphereType& bnds);
    virtual void aggregateDestroyed(AggregatorType* handler, const ObjectIDType& objid);
    virtual void aggregateObserved(AggregatorType* handler, const ObjectIDType& objid, uint32 nobservers);

    void drawbb(const BoundingBox3& bb);
    void drawbs(const BoundingSphere& bs);

    void validateSeenObjects();

    SimulatorBase* mSimulatorBase;

    bool mDisplay;

    typedef std::tr1::unordered_map<ObjectID, uint32, ObjectID::Hasher> ObjectRefCountMap;
    ObjectRefCountMap mSeenObjects;
    int mWinWidth, mWinHeight;

    // Gets bounds of an object or aggregate object
    BoundingSphere getBounds(const ObjectID& obj);

    typedef std::tr1::unordered_set<ObjectID, ObjectID::Hasher> ObjectIDSet;
    typedef std::tr1::unordered_map<ObjectID, ObjectIDSet, ObjectID::Hasher> AggregateObjectMap;
    AggregateObjectMap mAggregateObjects;

    uint32 mMaxObservers;

    typedef boost::mutex Mutex;
    typedef boost::lock_guard<Mutex> Lock;
    Mutex mAggregateMutex;
    typedef std::tr1::unordered_map<ObjectID, BoundingSphereType, ObjectID::Hasher> AggregateBounds;
    AggregateBounds mAggregateBounds;

    enum DisplayMode {
        TimesSeen,
        SmallestAggregates,
        NumDisplayModes
    };
    DisplayMode mDisplayMode;

private:
    GLRendererBase();
}; // class GLRendererBase

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIMCORE_GLRENDERERBASE_HPP_
