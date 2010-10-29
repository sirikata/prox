/*  libprox
 *  Range.hpp
 *
 *  Copyright (c) 2010, Ewen Cheslack-Postava
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

#ifndef _PROX_RANGE_HPP_
#define _PROX_RANGE_HPP_

namespace Prox {

struct Range {
    uint32 start, end;

    Range(uint32 _sz)
     : start(0), end(_sz-1)
    {}
    Range(uint32 _start, uint32 _end)
     : start(_start), end(_end)
    {}

    Range bottom() const {
        assert(!single());
        return Range(start, (start+end)/2);
    }
    Range top() const {
        assert(!single());
        return Range((start+end)/2+1, end);
    };

    uint32 size() const { return end - start + 1; }

    uint32 single() const { return size() == 1; }
};

/** Maps a range of nodes to parent (or some ancestor) nodes approximately
 *  evenly.
 */
struct RangeMapping {
    // To get all the range right, even with subdivisions, we track to
    // total number of children and grandchildren, as well as the
    // start and end of the current child range, in integral
    // divisions.  From these, we can split correctly as well as
    // compute the exact ranges we for children and grandchildren.
    uint32 childCount;
    uint32 grandchildCount;

    uint32 divStart;
    uint32 divEnd;

public:
    RangeMapping(
        uint32 cc, uint32 gcc,
        uint32 ds, uint32 de
    )
     : childCount(cc), grandchildCount(gcc),
       divStart(ds), divEnd(de)
    {
        assert(ds < cc);
        assert(de < cc);
        assert(ds <= de);
    }

    uint32 child() const {
        assert(singleDiv());
        return divStart;
    }

    uint32 grandchildStart() const {
        uint32 retval = (divStart*grandchildCount)/childCount;
        assert(retval >= 0);
        assert(retval < grandchildCount);
        return retval;
    }
    uint32 grandchildEnd() const {
        uint32 retval = ((divEnd+1)*grandchildCount)/childCount - 1;
        assert(retval >= 0);
        assert(retval < grandchildCount);
        return retval;
    }

    bool singleDiv() const { return divStart == divEnd; }

    RangeMapping bottomHalf() const {
        return RangeMapping(
            childCount, grandchildCount,
            divStart, (divStart+divEnd)/2
        );
    }

    RangeMapping topHalf() const {
        return RangeMapping(
            childCount, grandchildCount,
            (divStart+divEnd)/2+1, divEnd
        );
    }
};


} // namespace Prox

#endif //_PROX_RANGE_HPP_
