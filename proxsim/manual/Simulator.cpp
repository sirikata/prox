// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "Simulator.hpp"
#include <proxsimcore/RandomUtils.hpp>
#include "CSVLoader.hpp"

#include <iostream>

namespace Prox {
namespace Simulation {

Simulator::Simulator(ManualQueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime)
 : SimulatorBase(duration, timestep, iterations, realtime),
   mHandler(handler)
{
}

Simulator::~Simulator() {
}

void Simulator::initialize(int churnrate) {
    SimulatorBase::initialize(churnrate);
    mHandler->initialize(mLocCache, mLocCache, !mHaveMovingObjects);
}

void Simulator::shutdown() {
    delete mHandler;
    mHandler = NULL;

    SimulatorBase::shutdown();
}

static Querier* generateQuery(ManualQueryHandler* handler, const BoundingBox3& region, bool static_queries) {
    Vector3 qpos = generatePosition(region);
    Vector3 qvel = generateDirection(!static_queries);

    MotionPath::MotionVectorListPtr mpl(new MotionPath::MotionVectorList());
    mpl->push_back(MotionVector3(Time::null(), qpos, qvel));

    Querier* querier = new Querier(handler,
        MotionPath(Vector3(0,0,0), mpl),
        generateQueryBounds(),
        generateQueryRadius()
    );
    return querier;
}

void Simulator::createRandomQueries(int nqueries, bool static_queries) {
    for(int i = 0; i < nqueries; i++)
        addQuery( generateQuery(mHandler, mRegion, static_queries) );
}

void Simulator::createCSVQueries(int nqueries, const std::string& csvmotionfile) {
    float qradius = generateQueryRadius();

    std::vector<Querier*> qs =
        loadCSVMotionQueriers(
            csvmotionfile, nqueries,
            mHandler,
            std::tr1::bind(generatePosition, mRegion),
            qradius
        );
    for(int i = 0; i < (int)qs.size(); i++)
        addQuery(qs[i]);
}

void Simulator::tick_work(Time last_time, Duration elapsed) {
    SimulatorBase::tick_work(last_time, elapsed);

    // Give all queries a chance to update
    for(QueryList::iterator it = mQueries.begin(); it != mQueries.end(); it++)
        (*it)->tick(mTime);

    mHandler->tick(mTime);
}

void Simulator::printNodes() const {
    for(QueryHandler::NodeIterator nit = mHandler->nodesBegin(); nit != mHandler->nodesEnd(); nit++)
        std::cout << nit.id().toString() << std::endl;
    std::cout << std::endl << std::endl;
}

void Simulator::addListener(SimulatorQueryListener* listener) {
    assert( std::find(mQueryListeners.begin(), mQueryListeners.end(), listener) == mQueryListeners.end() );
    mQueryListeners.push_back(listener);
}

void Simulator::removeListener(SimulatorQueryListener* listener) {
    QueryListenerList::iterator it = std::find(mQueryListeners.begin(), mQueryListeners.end(), listener);
    assert( it != mQueryListeners.end() );
    mQueryListeners.erase(it);
}

void Simulator::addQuery(Querier* query) {
    mQueries.push_back(query);
    for(QueryListenerList::iterator it = mQueryListeners.begin(); it != mQueryListeners.end(); it++)
        (*it)->simulatorAddedQuery(query);
}

void Simulator::removeQuery(Querier* query) {
    QueryList::iterator it = std::find(mQueries.begin(), mQueries.end(), query);
    mQueries.erase(it);

    for(QueryListenerList::iterator it = mQueryListeners.begin(); it != mQueryListeners.end(); it++)
        (*it)->simulatorRemovedQuery(query);
}

Simulator::QueryIterator Simulator::queriesBegin() {
    return mQueries.begin();
}

Simulator::QueryIterator Simulator::queriesEnd() {
    return mQueries.end();
}

uint32 Simulator::queriesSize() const {
    return mQueries.size();
}

} // namespace Simulation
} // namespace Prox
