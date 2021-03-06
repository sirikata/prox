/*  libprox
 *  DefaultSimulationTraits.hpp
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

#ifndef _PROX_DEFAULT_SIMULATION_TRAITS_HPP_
#define _PROX_DEFAULT_SIMULATION_TRAITS_HPP_

#include <prox/util/Vector3.hpp>
#include <prox/util/MotionVector.hpp>

#include <prox/util/BoundingSphere.hpp>

#include <prox/util/SolidAngle.hpp>

#include <prox/util/ObjectID.hpp>

#include <prox/util/Time.hpp>
#include <prox/util/Duration.hpp>

#include <prox/util/UniqueID.hpp>

// LocationServiceCache?

namespace Prox {

class DefaultSimulationTraits {
public:
    typedef float realType;

    typedef Reference::Vector3f Vector3Type;
    typedef Reference::MotionVector3f MotionVector3Type;

    typedef Reference::BoundingSphere3f BoundingSphereType;

    typedef Reference::SolidAngle SolidAngleType;

    typedef Reference::ObjectID ObjectIDType;
    typedef Reference::ObjectID::Hasher ObjectIDHasherType;
    typedef Reference::ObjectID::Null ObjectIDNullType;
    typedef Reference::ObjectID::Random ObjectIDRandomType;

    typedef Reference::Time TimeType;
    typedef Reference::Duration DurationType;

    const static realType InfiniteRadius;

    // Flags an infinite number of results may be returned, the default
    const static uint32 InfiniteResults;

    typedef Reference::UniqueIDGenerator UniqueIDGeneratorType;
}; // class DefaultSimulationTraits

} // namespace Prox

#endif //_PROX_DEFAULT_SIMULATION_TRAITS_HPP_
