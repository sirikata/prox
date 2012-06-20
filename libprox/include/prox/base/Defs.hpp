// Copyright (c) 2011. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _PROX_BASE_DEFS_HPP_
#define _PROX_BASE_DEFS_HPP_

#include <prox/util/Platform.hpp>

namespace Prox {

// Identifier for an index within a QueryHandler, e.g. a single tree. A query
// handler may have more than one (e.g. during rebuilding) causing multiple
// copies of an object to appear. This ID can be used to uniquely identify a
// copy.
typedef uint32 QueryHandlerIndexID;

} // namespace Prox

#endif //_PROX_BASE_DEFS_HPP_
