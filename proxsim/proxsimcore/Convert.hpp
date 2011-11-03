// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_PROXSIMCORE_CONVERT_HPP_
#define _PROX_PROXSIMCORE_CONVERT_HPP_

#include <boost/lexical_cast.hpp>

static bool convert_bool(const std::string& arg) {
    return (arg == "on" || arg == "true" || arg == "yes");
}

template<typename T>
void convert_range(const std::string& arg, T* rmin, T* rmax) {
    if (arg.find("(") != std::string::npos) {
        // A real range of the format "(x,y)"
        int lparen = arg.find("(");
        int comma = arg.find(",");
        int rparen = arg.find(")");
        assert((std::size_t)lparen != std::string::npos);
        assert((std::size_t)comma != std::string::npos);
        assert((std::size_t)rparen != std::string::npos);
        std::string first = arg.substr( lparen+1, (comma-(lparen+1)) );
        std::string second = arg.substr( comma+1, (rparen-(comma+1)) );
        *rmin = boost::lexical_cast<T>(first);
        *rmax = boost::lexical_cast<T>(second);
    }
    else {
        // Just a single value
        T val = boost::lexical_cast<T>(arg);
        *rmin = val;
        *rmax = val;
    }
}

#endif //_PROX_PROXSIMCORE_CONVERT_HPP_
