/*  Sirikata
 *  CSVLoader.cpp
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

#ifndef _PROXSIMCORE_CSVLOADER_HPP_
#define _PROXSIMCORE_CSVLOADER_HPP_

#include <proxsimcore/Object.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>

namespace Prox {
namespace Simulation {

struct MotionAndBounds {
    MotionPath::MotionVectorListPtr motion;
    Vector3 position;
    float radius;
};

struct MotionInfo {
    std::vector<Vector3> positions;
    std::vector<Time> times;
    float rad;
};

std::vector<MotionAndBounds> loadCSVMotions(const String& filename);
std::vector<Object*> loadCSVObjects(const String& filename);
std::vector<Object*> loadCSVMotionObjects(const String& filename, std::tr1::function<Vector3()> gen_loc, int nobjects);

} // namespace Simulation
} // namespace Prox

#endif //_PROXSIMCORE_CSVLOADER_HPP_
