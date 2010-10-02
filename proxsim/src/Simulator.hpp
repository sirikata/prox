/*  proxsim
 *  Simulator.hpp
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

#ifndef _PROXSIM_SIMULATOR_HPP_
#define _PROXSIM_SIMULATOR_HPP_

#include "SimulationTypes.hpp"
#include "BoundingBox.hpp"
#include "Object.hpp"
#include "SimulatorListener.hpp"
#include "ObjectLocationServiceCache.hpp"
#include "Timer.hpp"

namespace Prox {
namespace Simulation {

class Simulator {
private:
    typedef std::tr1::unordered_map<ObjectID, Object*, ObjectID::Hasher> ObjectList;
    typedef std::list<Query*> QueryList;
public:
    Simulator(QueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime);
    ~Simulator();

    void initialize(const BoundingBox3& region, int nobjects, float moving_frac, int nqueries, bool static_queries, int churnrate);
    void shutdown();

    const BoundingBox3& region() const;

    void addListener(SimulatorListener* listener);
    void removeListener(SimulatorListener* listener);

    Time time() const { return mTime; }
    bool finished() const { return mFinished; }

    void printRate(bool p) { mReportRate = p; }

    void tick();

    typedef ObjectList::iterator ObjectIterator;
    typedef QueryList::iterator QueryIterator;

    ObjectIterator objectsBegin();
    ObjectIterator objectsEnd();
    ObjectIterator objectsFind(const ObjectID& objid);

    QueryIterator queriesBegin();
    QueryIterator queriesEnd();
    uint32 queriesSize() const;

private:
    void addObject(Object* obj);
    void removeObject(Object* obj);

    void addQuery(Query* query);
    void removeQuery(Query* query);

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
    QueryHandler* mHandler;
    ObjectLocationServiceCache* mLocCache;
    ObjectList mAllObjects;
    ObjectList mObjects;
    QueryList mQueries;
    typedef std::list<SimulatorListener*> ListenerList;
    ListenerList mListeners;

    ObjectList mRemovedObjects;
    int32 mChurn; // Rate at which objects are added and removed

    bool mReportRate;
    int32 mItsSinceRateApprox;
    Duration mRateApproxStart;
}; // class Simulator

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_SIMULATOR_HPP_
