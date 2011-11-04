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

#include <proxsimcore/SimulatorBase.hpp>
#include "SimulatorQueryListener.hpp"
#include "Querier.hpp"

namespace Prox {
namespace Simulation {

class Simulator : public SimulatorBase {
private:
    typedef std::list<Querier*> QueryList;
public:
    Simulator(QueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime);
    ~Simulator();

    void initialize(int churnrate, const SolidAngle& min_qangle, const SolidAngle& max_qangle, const float dist, uint32 max_results);

    // note: call these after initialize
    void createRandomQueries(int nqueries, bool static_queries);
    void createCSVQueries(int nqueries, const std::string& csvmotionfile);

    virtual void shutdown();

    void addListener(SimulatorQueryListener* listener);
    void removeListener(SimulatorQueryListener* listener);

    void printRate(bool p) { mReportRate = p; }
    void forceInitialRebuild(bool p) { mForceInitialRebuild = p; }
    void forceRebuild(bool p) { mForceRebuild = p; }

    virtual void tick_work(Time last_time, Duration elapsed);

    typedef QueryList::iterator QueryIterator;

    QueryIterator queriesBegin();
    QueryIterator queriesEnd();
    uint32 queriesSize() const;

private:
    void addQuery(Querier* query);
    void removeQuery(Querier* query);

    QueryHandler* mHandler;

    SolidAngle mQueryAngleMin;
    SolidAngle mQueryAngleMax;
    float mQueryDistance;
    uint32 mQueryMaxResults;
    QueryList mQueries;

    typedef std::list<SimulatorQueryListener*> QueryListenerList;
    QueryListenerList mQueryListeners;

    bool mForceInitialRebuild; // Force rebuild on initial tick
    bool mForceRebuild; // Force rebuild every frame by removing all objects and
                        // adding them back in again.
    bool mReportRate;
    int32 mItsSinceRateApprox;
    Duration mRateApproxStart;
}; // class Simulator

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_SIMULATOR_HPP_
