/*  proxsim
 *  MotionPath.hpp
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

#ifndef _PROXSIM_MOTION_PATH_
#define _PROXSIM_MOTION_PATH_

#include "SimulationTypes.hpp"
#include <stdio.h>

namespace Prox {
namespace Simulation {

/** A motion path is a sequence of motion updates and tracks the current state
 * of a .  The underlying sequence of updates is shared.
 */
class MotionPath {
public:
    typedef std::vector<MotionVector3> MotionVectorList;
    typedef std::tr1::shared_ptr<MotionVectorList> MotionVectorListPtr;

    /** Create a new MotionPath using the given updates as a template, and
     *  offset by the specified vector.
     */
    MotionPath(const Vector3& offset, const MotionVectorListPtr& updates, bool loop = false)
     : mOffset(offset),
       mLoop(loop),
       mMotionList(updates),
       mLength( updates->at(updates->size()-1).updateTime() - Time::null() ),
       mCurPosition(updates->begin()),
       mCurrent((*mCurPosition + mOffset))
    {
        assert(mLoop == false || !(mLength == Duration(0)));
    }

    // Process the MotionVector up to the given time and provide the motion
    // vector for that time. Returns true if the position + velocity setting
    // changed (i.e. current() is different before and after this call).
    bool tick(const Time& t) {
        bool changed = false;

        // Figure out where in the loop we are
        Time t_loop(t);
        if (mLoop) {
            int64 microsecs = (t-Time::null()).microseconds() % mLength.microseconds();
            t_loop = (Time::null() + Duration::microseconds(microsecs));

            if (mCurPosition->updateTime() > t_loop) {
                mCurPosition = mMotionList->begin();
                changed = true;
            }
        }

        while(true) {
            MotionVectorList::iterator next_ = mCurPosition + 1;
            if (next_ == mMotionList->end() ||
                next_->updateTime() > t_loop)
                break;
            mCurPosition = next_;
            changed = true;
        }
        // Interpolate and build a new MotionVector based on the looped data and
        // current time
        mCurrent = MotionVector3(
            t,
            mOffset + (mCurPosition->position(t_loop)),
            mCurPosition->velocity()
        );
        return changed;
    }

    const MotionVector3& current() const {
        return mCurrent;
    }

private:
    MotionPath();

    Vector3 mOffset;
    bool mLoop;
    MotionVectorListPtr mMotionList;
    Duration mLength;
    MotionVectorList::iterator mCurPosition;
    MotionVector3 mCurrent;
};

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIM_MOTION_PATH_
