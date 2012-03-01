// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_SIMULATOR_HPP_
#define _PROXSIM_MANUAL_SIMULATOR_HPP_

#include <proxsimcore/SimulatorBase.hpp>
#include "Querier.hpp"
#include "SimulatorQueryListener.hpp"

namespace Prox {
namespace Simulation {

class Simulator : public SimulatorBase {
private:
    typedef std::list<Querier*> QueryList;
public:
    Simulator(ManualQueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime);
    ~Simulator();

    void initialize(int churnrate);

    // note: call these after initialize
    void createRandomQueries(int nqueries, bool static_queries);
    void createCSVQueries(int nqueries, const std::string& csvmotionfile);

    virtual void shutdown();

    void addListener(SimulatorQueryListener* listener);
    void removeListener(SimulatorQueryListener* listener);

    virtual void tick_work(Time last_time, Duration elapsed);

    virtual void printNodes() const;

    typedef QueryList::iterator QueryIterator;

    QueryIterator queriesBegin();
    QueryIterator queriesEnd();
    uint32 queriesSize() const;

private:
    void addQuery(Querier* query);
    void removeQuery(Querier* query);

    ManualQueryHandler* mHandler;

    QueryList mQueries;

    typedef std::list<SimulatorQueryListener*> QueryListenerList;
    QueryListenerList mQueryListeners;

}; // class Simulator

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_SIMULATOR_HPP_
