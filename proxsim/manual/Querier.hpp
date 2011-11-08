// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROXSIM_MANUAL_QUERIER_HPP_
#define _PROXSIM_MANUAL_QUERIER_HPP_

#include <proxsimcore/SimulationTypes.hpp>
#include <proxsimcore/MotionPath.hpp>

namespace Prox {
namespace Simulation {

class Querier : public ManualQueryEventListener {
public:
    Querier(ManualQueryHandler* handler, const MotionPath& mp, const BoundingSphere& bounds, float max_querier_radius);
    ~Querier();

    void tick(const Time& t);

    // Pass through calls for Query
    void setEventListener(ManualQueryEventListener* listener) {
        // We intercept and forward these so we have a set of objects we can
        // refine/coarsen. However, we only do this in response to the parent
        // listener so the parent doesn't lose any events.
        mQueryEventListener = listener;
        mQuery->setEventListener( (listener == NULL ? NULL : this) );
    }
    Vector3 position(const Time& t) const {
        return mQuery->position(t);
    }

    virtual void queryHasEvents(ManualQuery* query);
private:
    MotionPath mMotion;
    ManualQuery* mQuery;
    ManualQueryEventListener* mQueryEventListener;
    // Use a set so we can select from it randomly
    typedef std::set<ObjectID> ObjectSet;
    ObjectSet mResults;
};

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MANUAL_QUERIER_HPP_
