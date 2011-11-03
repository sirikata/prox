/*  proxsim
 *  Simulator.cpp
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

#include "Simulator.hpp"
#include <stdint.h>
#include <algorithm>
#include "CSVLoader.hpp"

#define RATE_APPROX_ITERATIONS 25

namespace Prox {
namespace Simulation {

static float randFloat() {
    return float(rand()) / RAND_MAX;
}

static uint32 randUInt32(uint32 minval, uint32 maxval) {
    uint32 r = (uint32)( randFloat() * (maxval-minval+1) + minval);
    if (r > maxval) r = maxval;
    return r;
}

Simulator::Simulator(QueryHandler* handler, int duration, const Duration& timestep, int iterations, bool realtime)
 : SimulatorBase(duration, timestep, iterations, realtime),
   mHandler(handler),
   mForceInitialRebuild(false),
   mForceRebuild(false),
   mReportRate(false),
   mItsSinceRateApprox(0),
   mRateApproxStart(0)
{
}

Simulator::~Simulator() {
}

static Vector3 generatePosition(const BoundingBox3& region) {
    Vector3 region_min = region.min();
    Vector3 region_extents = region.extents();

    return region_min + Vector3(region_extents.x * randFloat(), region_extents.y * randFloat(), 0.f/*region_extents.z * randFloat()*/);
}

static Vector3 generateDirection(bool moving) {
    return
        (moving ?
            Vector3(randFloat() * 20.f - 10.f, randFloat() * 20.f - 10.f, 0.f/*randFloat() * 20.f - 10.f*/) :
            Vector3(0, 0, 0)
        );
};

static MotionPath generateMotionPath(const BoundingBox3& region, bool moving) {
    // Our simple model is ballistic, single update

    Vector3 offset = generatePosition(region);
    MotionPath::MotionVectorListPtr updates(
        new MotionPath::MotionVectorList()
    );
    Vector3 dir = generateDirection(moving);
    updates->push_back(
        MotionVector3(
            Time::null(),
            Vector3(0,0,0),
            dir
        )
    );

    return MotionPath(offset, updates);
}

static BoundingBox3 generateObjectBounds() {
    return BoundingBox3( Vector3(-1, -1, -1), Vector3(1, 1, 1));
};

static BoundingBox3 generateQueryBounds() {
    return BoundingBox3( Vector3(0, 0, 0), Vector3(0, 0, 0) );
}

static float generateQueryRadius() {
    static float val = sqrtf(6.f) / 2.f; // Diagonal of bounding box
    return val;
}

static SolidAngle generateQueryAngle(const SolidAngle& qmin, const SolidAngle& qmax) {
    assert(qmax >= qmin);
    if (qmax == qmin) return qmin;

    return qmin + ((qmax-qmin) * (((float)(rand()))/RAND_MAX));
}

static Querier* generateQuery(QueryHandler* handler, const BoundingBox3& region, bool static_queries, const SolidAngle& qmin, const SolidAngle& qmax, const float q_distance, uint32 q_max_results) {
    Vector3 qpos = generatePosition(region);
    Vector3 qvel = generateDirection(!static_queries);

    MotionPath::MotionVectorListPtr mpl(new MotionPath::MotionVectorList());
    mpl->push_back(MotionVector3(Time::null(), qpos, qvel));

    Querier* querier = new Querier(handler,
        MotionPath(Vector3(0,0,0), mpl),
        generateQueryBounds(),
        generateQueryRadius(),
        generateQueryAngle(qmin, qmax),
        q_distance,
        q_max_results
    );
    return querier;
}

void Simulator::initialize(int churnrate, const SolidAngle& min_qangle, const SolidAngle& max_qangle, const float dist, uint32 max_results) {
    SimulatorBase::initialize(churnrate);

    mQueryAngleMin = min_qangle;
    mQueryAngleMax = max_qangle;
    mQueryDistance = dist;
    mQueryMaxResults = max_results;

    mHandler->initialize(mLocCache, mLocCache, !mHaveMovingObjects);
}

void Simulator::createRandomQueries(int nqueries, bool static_queries) {
    for(int i = 0; i < nqueries; i++)
        addQuery( generateQuery(mHandler, mRegion, static_queries, mQueryAngleMin, mQueryAngleMax, mQueryDistance, mQueryMaxResults) );
}

void Simulator::createCSVQueries(int nqueries, const std::string& csvmotionfile) {
    float qradius = generateQueryRadius();
    SolidAngle qangle = generateQueryAngle(mQueryAngleMin, mQueryAngleMax);
    float qdistance = mQueryDistance;

    std::vector<Querier*> qs =
        loadCSVMotionQueriers(
            csvmotionfile, nqueries,
            mHandler,
            std::tr1::bind(generatePosition, mRegion),
            qradius, qangle, qdistance
        );
    for(int i = 0; i < (int)qs.size(); i++)
        addQuery(qs[i]);
}

void Simulator::shutdown() {
    while(!mQueries.empty()) {
        Querier* query = mQueries.front();
        removeQuery(query);
        delete query;
    }

    delete mHandler;
    mHandler = NULL;

    SimulatorBase::shutdown();
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

void Simulator::tick_work(Time last_time, Duration elapsed) {
    SimulatorBase::tick_work(last_time, elapsed);

    if (mForceRebuild || (mForceInitialRebuild && last_time == Time::null())) {
        mHandler->rebuild();
    }
    else {
/*
        // Object Churn...
        for(int i = 0; !mObjects.empty() && i < mChurn; i++) {
            Object* obj = mObjects.begin()->second;
            removeObject(obj);
        }
        for(int i = 0; !mRemovedObjects.empty() && i < mChurn; i++) {
            Object* obj = mRemovedObjects.begin()->second;
            addObject(obj);
        }
*/
    }

    // Give all queries a chance to update
    for(QueryList::iterator it = mQueries.begin(); it != mQueries.end(); it++)
        (*it)->tick(mTime);

    // Tick the handler
    if (mForceRebuild || last_time == Time::null()) {
        // To simplify stats collection with rebuilds, we allow for
        // double-ticking.  We use the same timestep and just.  This just lets
        // us get valid "checks" counts when we're rebuilding every frame.
        // We also do this always on the first tick to avoid reporting the
        // initial cost of pushing down a cut.
        mHandler->tick(mTime, false);
        mHandler->tick(mTime, true);
    }
    else {
        // normal ticking
        mHandler->tick(mTime);
    }

    mItsSinceRateApprox++;
    if (mItsSinceRateApprox >= RATE_APPROX_ITERATIONS) {
        float its_per_sec = float(RATE_APPROX_ITERATIONS) * queriesSize() / (elapsed - mRateApproxStart).seconds();
        if (mReportRate)
            printf("{ \"rate\" : %f }\n", its_per_sec);
        // Reset for next round
        mItsSinceRateApprox = 0;
        mRateApproxStart = elapsed;
    }
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
