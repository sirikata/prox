// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#include "Querier.hpp"
#include <iostream>

namespace Prox {
namespace Simulation {

Querier::Querier(ManualQueryHandler* handler, const MotionPath& mp, const BoundingSphere& bounds, float max_querier_radius)
 : mMotion(mp)
{
    mQuery = handler->registerQuery(
        mp.current(),
        bounds, max_querier_radius
    );
}

Querier::~Querier() {
    delete mQuery;
}

void Querier::queryHasEvents(ManualQuery* query) {
    // Track object set
    std::deque<QueryEvent> evts;
    query->copyEvents(evts);
    for(std::deque<QueryEvent>::iterator it = evts.begin(); it != evts.end(); it++) {
        for(QueryEvent::AdditionList::iterator add_it = it->additions().begin(); add_it != it->additions().end(); add_it++)
            mResults.insert(add_it->id());
        for(QueryEvent::RemovalList::iterator rem_it = it->removals().begin(); rem_it != it->removals().end(); rem_it++)
            mResults.erase(rem_it->id());
    }

    // Forward the event
    mQueryEventListener->queryHasEvents(query);
}

void Querier::tick(const Time& t) {
    bool changed = mMotion.tick(t);
    if (changed)
        mQuery->position(mMotion.current());

    // Pick random result and try to re
    ObjectID rand_id = ObjectID::Random()();
    ObjectSet::iterator it = std::lower_bound(mResults.begin(), mResults.end(), rand_id);
    if (it == mResults.end()) return;
    ObjectID rand_obj = *it;

    // Choose whether the refine or coarsen
    bool refine = ((rand() % 2) == 0);
    // Special cases to ensure we don't get a bad cut (too big or too small)
    if (mResults.size() < 5)
        refine = true;
    else if (mResults.size() > 50)
        refine = false;

    if (refine)
        mQuery->refine(rand_obj);
    else
        mQuery->coarsen(rand_obj);

}

} // namespace Simulation
} // namespace Prox
