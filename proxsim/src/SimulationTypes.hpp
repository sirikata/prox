/*  proxsim
 *  Object.hpp
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

#ifndef _PROX_SIMULATION_TYPES_HPP_
#define _PROX_SIMULATION_TYPES_HPP_

#include <prox/base/DefaultSimulationTraits.hpp>
#include <prox/geom/Query.hpp>
#include <prox/base/QueryEvent.hpp>
#include <prox/base/QueryEventListener.hpp>
#include <prox/base/LocationServiceCache.hpp>
#include <prox/base/AggregateListener.hpp>
#include <prox/geom/QueryHandler.hpp>

namespace Prox {
namespace Simulation {

// typedefs for the types we want to use in the Simulation so we don't have
// to refer to them in a different namespapce / struct.

typedef Prox::DefaultSimulationTraits::realType real;

typedef Prox::DefaultSimulationTraits::Vector3Type Vector3;
typedef Prox::DefaultSimulationTraits::MotionVector3Type MotionVector3;

typedef Prox::DefaultSimulationTraits::BoundingSphereType BoundingSphere;

typedef Prox::DefaultSimulationTraits::SolidAngleType SolidAngle;

typedef Prox::DefaultSimulationTraits::ObjectIDType ObjectID;

typedef Prox::DefaultSimulationTraits::TimeType Time;
typedef Prox::DefaultSimulationTraits::DurationType Duration;


typedef Prox::Query<DefaultSimulationTraits> Query;
typedef Prox::QueryEvent<DefaultSimulationTraits> QueryEvent;
typedef Prox::QueryEventListener<DefaultSimulationTraits, Query> QueryEventListener;
typedef Prox::QueryHandler<DefaultSimulationTraits> QueryHandler;

typedef Prox::LocationServiceCache<DefaultSimulationTraits> LocationServiceCache;

typedef Prox::AggregateListener<DefaultSimulationTraits> AggregateListener;

} // namespace Simulation
} // namespace Prox

#endif //_PROX_SIMULATION_TYPES_HPP_
