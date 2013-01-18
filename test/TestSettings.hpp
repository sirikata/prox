// Copyright (c) 2013. All rights reserved.
// Use of this source code is governed by a BSD-style license that can
// be found in the LICENSE file.

#ifndef _LIBPROX_TEST_TEST_SETTINGS_HPP_
#define _LIBPROX_TEST_TEST_SETTINGS_HPP_

#include <prox/rtree/RTree.hpp>

#ifndef LIBPROX_RTREE_DATA
# error "You must define LIBPROX_RTREE_DATA to either LIBPROX_RTREE_DATA_BOUNDS or LIBPROX_RTREE_DATA_MAXSIZE"
#endif
#if LIBPROX_RTREE_DATA == LIBPROX_RTREE_DATA_BOUNDS
typedef Prox::BoundingSphereData<Prox::DefaultSimulationTraits> TestNodeData;
#elif LIBPROX_RTREE_DATA == LIBPROX_RTREE_DATA_MAXSIZE
typedef Prox::MaxSphereData<Prox::DefaultSimulationTraits> TestNodeData;
#elif LIBPROX_RTREE_DATA == LIBPROX_RTREE_DATA_SIMILARMAXSIZE
typedef Prox::SimilarMaxSphereData<Prox::DefaultSimulationTraits> TestNodeData;
#else
# error "Invalid setting for LIBPROX_RTREE_DATA"
#endif

#endif //_LIBPROX_TEST_TEST_SETTINGS_HPP_
