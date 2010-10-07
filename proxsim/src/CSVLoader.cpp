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

#include "CSVLoader.hpp"
#include <boost/lexical_cast.hpp>
#include <fstream>

namespace Prox {
namespace Simulation {

template<typename T>
T safeLexicalCast(const String& orig, T default_val) {
    if (orig.empty())
        return default_val;
    return boost::lexical_cast<T>(orig);
}

template<typename T>
T safeLexicalCast(const String& orig) {
    return safeLexicalCast<T>(orig, (T)0);
}

namespace {
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
}

std::vector<MotionAndBounds> loadCSVMotions(const String& filename) {
    std::vector<MotionAndBounds> results;

    std::ifstream fp(filename.c_str());
    if (!fp) return results;

    typedef std::map<std::string, MotionInfo> MotionInfoMap;
    MotionInfoMap objects;

    // Read in and sort the data
    while(fp) {
        String line;
        std::getline(fp, line);

        char buf[128];
        float x, y, z, time_ms, rad;
        // format note: each line has objid: ..., but %s reads until
        // whitespace. we just include the : in the id...
        int matched = sscanf(line.c_str(), "%s %f, %f, %f, %f, %f",
            buf, &x, &y, &z, &time_ms, &rad
        );
        if (matched < 6) continue;

        MotionInfoMap::iterator it = objects.find(std::string(buf));
        if (it == objects.end()) {
            objects[std::string(buf)] = MotionInfo();
            it = objects.find(std::string(buf));
        }
        it->second.positions.push_back(
            Vector3(x, y, 0) // flatten
        );
        it->second.times.push_back(Time::null() + Duration::milliseconds(time_ms));
        it->second.rad = rad;
    }

    // Reconstruct paths for each object.
    for(MotionInfoMap::iterator it = objects.begin(); it != objects.end(); it++) {
        std::string name = it->first;
        MotionInfo& mi = it->second;

        // Check that we have things in order
        assert(mi.times.size() > 1);
        for(int i = 0; i < mi.times.size()-1; i++)
            assert(mi.times[i] < mi.times[i+1]);

        // Fix times so they start at 0
        {
            Time init_time = mi.times[0];
            for(int i = 0; i < mi.times.size(); i++)
                mi.times[i] = Time::null() + (mi.times[i] - init_time);
        }

        MotionPath::MotionVectorListPtr ml(new MotionPath::MotionVectorList());

        assert(mi.positions.size() == mi.times.size());

        Vector3 start_pos = mi.positions[0];

        // Get list of updates that we care about
        std::vector<int> update_indices;
        // Always use first
        update_indices.push_back(0);
        // Get middle updates at most once per second
        Time last_update_time = Time::null();
        for(unsigned int i = 1; i < mi.positions.size()-1; i++) {
            if ( (mi.times[i] - last_update_time).seconds() >= 1.f) {
                update_indices.push_back(i);
                last_update_time = mi.times[i];
            }
        }
        // And always use last
        update_indices.push_back(mi.positions.size()-1);

        // To make it loopable, copy in reverse (leaving out the last one so we
        // don't duplicate a position+velocity)
        for(int i = update_indices.size()-2; i >= 0; i--)
            update_indices.push_back( update_indices[i] );

        // Then we generate the actual sequence, using the timing information carefully.
        assert(update_indices.size() > 2);
        Time cur_t = Time::null();
        for(unsigned int i = 0; i < update_indices.size()-1; i++) {
            unsigned int idx1 = update_indices[i];
            unsigned int idx2 = update_indices[i+1];
            Duration tdiff = (mi.times[idx2]-mi.times[idx1]).abs();
            Vector3 vel = (mi.positions[idx2]-mi.positions[idx1])/tdiff.seconds();
            ml->push_back(
                MotionVector3(
                    cur_t,
                    mi.positions[idx1]-start_pos,
                    vel
                )
            );
            cur_t += tdiff;
        }

        MotionAndBounds mab;
        mab.motion = ml;
        mab.position = start_pos;
        mab.radius = mi.rad;
        results.push_back(mab);
    }
    return results;
}

std::vector<Object*> loadCSVMotionObjects(const String& filename, std::tr1::function<Vector3()> gen_loc) {
    std::vector<MotionAndBounds> data = loadCSVMotions(filename);
    std::vector<Object*> results;

    for(unsigned int i = 0; i < data.size(); i++) {
        results.push_back(
            new Object(
                ObjectID::Random()(),
                MotionPath(gen_loc(), data[i].motion, true),
                BoundingSphere(Vector3(0,0,0), data[i].radius)
            )
        );
    }

    return results;
}

std::vector<Querier*> loadCSVMotionQueriers(const String& filename, int nqueriers, QueryHandler* qh, std::tr1::function<Vector3()> gen_loc, float qradius, const SolidAngle& qangle) {
    std::vector<MotionAndBounds> data = loadCSVMotions(filename);
    std::vector<Querier*> results;

    for(unsigned int i = 0; i < std::min((int)data.size(), nqueriers); i++) {
        int data_idx = rand() % data.size();
        results.push_back(
            new Querier(qh,
                MotionPath(gen_loc(), data[data_idx].motion, true),
                BoundingSphere(Vector3(0,0,0), data[data_idx].radius),
                qradius,
                qangle
            )
        );
    }

    return results;
}

std::vector<Object*> loadCSVObjects(const String& filename) {
    std::vector<Object*> results;
    typedef std::vector<String> StringList;

    std::ifstream fp(filename.c_str());
    if (!fp) return results;

    bool is_first = true;
    int objtype_idx = -1;
    int pos_idx = -1;
    int orient_idx = -1;
    int vel_idx = -1;
    int mesh_idx = -1;

    int quat_vel_idx = -1;

    int scale_idx = -1;

    // For each line
    while(fp) {
        String line;
        std::getline(fp, line);

        // Split into parts
        StringList line_parts;
        int last_comma = -1;
        String::size_type next_comma = 0;
        while(next_comma != String::npos) {
            next_comma = line.find(',', last_comma+1);

            String next_val;
            if (next_comma == String::npos)
                next_val = line.substr(last_comma + 1);
            else
                next_val = line.substr(last_comma + 1, next_comma - (last_comma+1));

            // Remove quotes from beginning and end
            if (next_val.size() > 2 && next_val[0] == '"' && next_val[next_val.size()-1] == '"')
                next_val = next_val.substr(1, next_val.size() - 2);

            line_parts.push_back(next_val);

            last_comma = next_comma;
        }



        if (is_first) {
            for(uint32 idx = 0; idx < line_parts.size(); idx++) {
                if (line_parts[idx] == "objtype") objtype_idx = idx;
                if (line_parts[idx] == "pos_x") pos_idx = idx;
                if (line_parts[idx] == "orient_x") orient_idx = idx;
                if (line_parts[idx] == "vel_x") vel_idx = idx;
                if (line_parts[idx] == "meshURI") mesh_idx = idx;
                if (line_parts[idx] == "rot_axis_x") quat_vel_idx = idx;
                if (line_parts[idx] == "scale") scale_idx = idx;
            }

            is_first = false;
        }
        else {
            assert(objtype_idx != -1 && pos_idx != -1 && mesh_idx != -1);

            if (line_parts[objtype_idx] == "mesh") {
                Vector3 pos(
                    safeLexicalCast<double>(line_parts[pos_idx+0]),
                    safeLexicalCast<double>(line_parts[pos_idx+2]), // swap y,z
                    safeLexicalCast<double>(line_parts[pos_idx+1])
                );
/* We don't care about rotation for this simulation... Remember to swap y,z if added
                Quaternion orient =
                    orient_idx == -1 ?
                    Quaternion(0, 0, 0, 1) :
                    Quaternion(
                        safeLexicalCast<float>(line_parts[orient_idx+0]),
                        safeLexicalCast<float>(line_parts[orient_idx+1]),
                        safeLexicalCast<float>(line_parts[orient_idx+2]),
                        safeLexicalCast<float>(line_parts[orient_idx+3]),
                        Quaternion::XYZW());
*/
               Vector3 vel =
                    vel_idx == -1 ?
                    Vector3(0, 0, 0) :
                    Vector3(
                        safeLexicalCast<float>(line_parts[vel_idx+0]),
                        safeLexicalCast<float>(line_parts[vel_idx+2]), // swap y,z
                        safeLexicalCast<float>(line_parts[vel_idx+1])
                    );
/*
                Vector3 rot_axis =
                    quat_vel_idx == -1 ?
                    Vector3(0, 0, 0) :
                    Vector3(
                        safeLexicalCast<float>(line_parts[quat_vel_idx+0]),
                        safeLexicalCast<float>(line_parts[quat_vel_idx+1]),
                        safeLexicalCast<float>(line_parts[quat_vel_idx+2])
                    );

                float angular_speed =
                    quat_vel_idx == -1 ?
                    0 :
                    safeLexicalCast<float>(line_parts[quat_vel_idx+3]);

                String mesh( line_parts[mesh_idx] );
*/
                float scale =
                    scale_idx == -1 ?
                    1.f :
                    safeLexicalCast<float>(line_parts[scale_idx], 1.f);


                // Construct a MotionPath
                MotionPath::MotionVectorListPtr mvl(
                    new MotionPath::MotionVectorList()
                );
                mvl->push_back(MotionVector3(Time::null(), Vector3(0,0,0), vel));

                results.push_back(
                    new Object(
                        ObjectID::Random()(),
                        MotionPath(pos, mvl),
                        BoundingSphere(Vector3(0,0,0), scale)
                    )
                );
            }
        }
    }

    fp.close();
    return results;
}

} // namespace Simulation
} // namespace Prox
