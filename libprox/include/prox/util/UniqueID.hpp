// Copyright (c) 2012. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_UTIL_UNIQUE_ID_HPP_
#define _PROX_UTIL_UNIQUE_ID_HPP_

#include <prox/util/Platform.hpp>

namespace Prox {
namespace Reference {

class UniqueIDGenerator {
public:
    uint32 operator()() {
        uint32 res;
        do {
            res = ++sSource;
        } while (res == 0);
        return res;
    }

private:
    static uint32 sSource;
}; // class UniqueIDGenerator

} // namespace Reference
} // namespace Prox

#endif //_PROX_UTIL_UNIQUE_ID_HPP_
