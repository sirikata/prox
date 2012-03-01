// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIMCORE_SIMULATORBASE_HPP_
#define _PROXSIMCORE_SIMULATORBASE_HPP_

#include <proxsimcore/SimulationTypes.hpp>
#include <proxsimcore/BoundingBox.hpp>
#include <proxsimcore/Object.hpp>
#include <proxsimcore/SimulatorObjectListener.hpp>
#include <proxsimcore/ObjectLocationServiceCache.hpp>
#include <proxsimcore/Timer.hpp>

namespace Prox {
namespace Simulation {

class SimulatorBase {
private:
    typedef std::map<ObjectID, Object*> OrderedObjectList;
    typedef std::tr1::unordered_map<ObjectID, Object*, ObjectID::Hasher> ObjectList;
public:
    SimulatorBase(int duration, const Duration& timestep, int iterations, bool realtime);
    ~SimulatorBase();

    // note: call these before initialize
    void createRandomObjects(const BoundingBox3& region, int nobjects, float moving_frac);
    void createStaticCSVObjects(const std::string csvfile, int nobjects);
    void createMotionCSVObjects(const std::string csvfile, int nobjects);

    void initialize(int churnrate);

    // note: call this after all create and initialize calls
    virtual void run();

    virtual void shutdown();

    const BoundingBox3& region() const;

    void addListener(SimulatorObjectListener* listener);
    void removeListener(SimulatorObjectListener* listener);

    Time time() const { return mTime; }
    bool finished() const { return mFinished; }

    // Override tick_work to change the work performed each iteration.
    void tick();

    typedef ObjectList::iterator ObjectIterator;

    ObjectIterator objectsBegin();
    ObjectIterator objectsEnd();
    ObjectIterator objectsFind(const ObjectID& objid);
    int objectsSize() const;

    // Not all objects may be active
    int allObjectsSize() const;

    virtual void printNodes() const = 0;
protected:
    // Provides last_time and elapsed. Currnet time is available in mTime.
    virtual void tick_work(Time last_time, Duration elapsed);

    // Helper, filters the given list of objects to only add the given number of them.
    void createCSVObjects(std::vector<Object*>& objects, int nobjects);

    void addObject(Object* obj);
    void removeObject(Object* obj);

    // Reusable for different object loaders.
    void addObjects();

    bool mFinished;

    int mDuration;
    Duration mTimestep;
    int mIterations;
    int mTerminateIterations;
    bool mRealtime;

    Timer mTimer;
    Time mTime;

    BoundingBox3 mRegion;
    int64 mObjectIDSource;
    ObjectLocationServiceCache* mLocCache;
    bool mHaveMovingObjects;
    ObjectList mAllObjects;
    ObjectList mObjects;

    typedef std::list<SimulatorObjectListener*> ObjectListenerList;
    ObjectListenerList mObjectListeners;

    OrderedObjectList mRemovedStaticObjects;
    OrderedObjectList mRemovedDynamicObjects;
    int32 mChurn; // Rate at which objects are added and removed

}; // class SimulatorBase

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIMCORE_SIMULATORBASE_HPP_
