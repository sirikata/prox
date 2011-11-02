/*  libprox
 *  QueryCache.hpp
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

#ifndef _PROX_QUERY_CACHE_HPP_
#define _PROX_QUERY_CACHE_HPP_

#include <prox/base/QueryEvent.hpp>
#include <algorithm>

namespace Prox {

/** QueryCache stores the results of a query evaluation and makes it
 *  easy to generate QueryEvents by taking the 'difference' between
 *  two QueryCaches, i.e. the results of two query evaluations.
 *  QueryCache supports tracking only the top N objects by accepting a
 *  'score' for each result, evicting the lowest score object if the
 *  limit is reached. If no limit is placed on the cache size, most of
 *  the cost of this mode is avoided.
 */
template<typename SimulationTraits>
class QueryCache {
public:
    typedef typename SimulationTraits::ObjectIDType ObjectID;
    typedef typename SimulationTraits::ObjectIDHasherType ObjectIDHasher;
    typedef QueryEvent<SimulationTraits> QueryEventType;

    typedef std::tr1::unordered_set<ObjectID, ObjectIDHasher> ObjectIDSet;

    QueryCache(uint32 max_size)
     : mMaxSize(max_size)
    {
    }

    ~QueryCache() {
    }

    // Add an object with a score. Large scores are better, so you may need to
    // invert the values you compute when you assign scores.
    void add(const ObjectID& id, float32 score) {
        assert( mObjects.find(id) == mObjects.end() );
        mObjects.insert(id);

        if(mMaxSize != SimulationTraits::InfiniteResults) {
            mScoredObjects.push_back(ScoredObject(id, score));
            std::push_heap(mScoredObjects.begin(), mScoredObjects.end());
            clearOverflow();
        }
    }

    bool contains(const ObjectID& id) {
        return (mObjects.find(id) != mObjects.end());
    }

    void remove(const ObjectID& id) {
        assert( mObjects.find(id) != mObjects.end() );
        mObjects.erase(id);

        // This path is inefficient since we need to remove from the heap
        for(typename ScoredObjectHeap::iterator it = mScoredObjects.begin(); it != mScoredObjects.end(); it++) {
            if (it->second.id == id) {
                mScoredObjects.erase(it);
                break;
            }
        }
        std::make_heap(mScoredObjects.begin(), mScoredObjects.end());
    }

    void setMaxSize(uint32 max_size) {
        mMaxSize = max_size;
        clearOverflow();
    }

    /** Exchange a newer cache into this one, generating events as we go.
     *  \param newcache the new cache to replace this one with
     *  \param changes a queue provided by the caller to hold events
     *  \param permanent_removals a list of objects which have been permanently
     *         removed, passed along in the events.
     */
    void exchange(QueryCache& newcache, std::deque<QueryEventType>* changes, const ObjectIDSet& permanent_removals) {
        if (changes != NULL) {
            IDSet added_objs;
            std::set_difference(
                newcache.mObjects.begin(), newcache.mObjects.end(),
                mObjects.begin(), mObjects.end(),
                std::inserter(added_objs, added_objs.begin())
            );

            IDSet removed_objs;
            std::set_difference(
                mObjects.begin(), mObjects.end(),
                newcache.mObjects.begin(), newcache.mObjects.end(),
                std::inserter(removed_objs, removed_objs.begin())
            );

            // Approaches that use a QueryCache are not implementing imposters.
            // Therefore, we always have single action QueryEvents
            for(IDSetIterator it = added_objs.begin(); it != added_objs.end(); it++) {
                QueryEventType evt;
                evt.additions().push_back( typename QueryEventType::Addition(*it, QueryEventType::Normal) );
                changes->push_back(evt);
            }

            for(IDSetIterator it = removed_objs.begin(); it != removed_objs.end(); it++) {
                QueryEventType evt;
                typename QueryEventType::ObjectEventPermanence perm = QueryEventType::Transient;
                if (permanent_removals.find(*it) != permanent_removals.end()) perm = QueryEventType::Permanent;
                evt.removals().push_back( typename QueryEventType::Removal(*it, QueryEventType::Normal, perm) );
                changes->push_back(evt);
            }
        }

        mMaxSize = newcache.mMaxSize;
        mObjects = newcache.mObjects;
        mScoredObjects = newcache.mScoredObjects;
    }

    int size() const {
        return (int)mObjects.size();
    }

    bool full() const {
        return (mMaxSize != SimulationTraits::InfiniteResults && size() == (int)mMaxSize);
    }

    // Get the score of the worst object in the set. Only makes sense if not
    // using an infinite set of results (we don't track ordering if using an
    // infinite set).
    float32 minScore() const {
        assert(mMaxSize != SimulationTraits::InfiniteResults);
        assert(size() > 0);

        return mScoredObjects.front().score;
    }
private:
    // Must specify maximum size
    QueryCache() {
    }

    // Clear 'overflow' objects, i.e. the lowest scored objects until the size
    // doesn't exceed our maximum
    void clearOverflow() {
        if (mMaxSize == SimulationTraits::InfiniteResults) return;

        while(size() > (int)mMaxSize) {
            std::pop_heap(mScoredObjects.begin(), mScoredObjects.end());
            mObjects.erase(mScoredObjects.back().id);
            mScoredObjects.pop_back();
        }
    }

    uint32 mMaxSize;

    typedef std::set<ObjectID> IDSet;
    typedef typename IDSet::iterator IDSetIterator;
    IDSet mObjects;

    // Tracks scored objects in a heap (backed by a vector) so they
    // can be evicted if maximum size is reached. This storage is not
    // maintained if the maximum size is InfiniteResults.
    struct ScoredObject {
        ScoredObject(const ObjectID& oid, float32 sc)
         : id(oid), score(sc)
        {}

        ObjectID id;
        float32 score;

        // Using the STL heap methods with < gives you max heap, invert it so we
        // remove bad scores first.
        bool operator<(const ScoredObject& rhs) {
            return score > rhs.score;
        }
    };
    typedef std::vector<ScoredObject> ScoredObjectHeap;
    ScoredObjectHeap mScoredObjects;
}; // class QueryCache

} // namespace Prox

#endif //_PROX_QUERY_CACHE_HPP_
