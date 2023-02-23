/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

#ifdef RAI_FCL
#include <fcl/config.h>

#if FCL_MINOR_VERSION < 6  // version 0.5.x

namespace fcl {
class CollisionObject;
class DynamicAABBTreeCollisionManager;
class BroadPhaseCollisionManager;

using CollisionObjectd = CollisionObject;
using DynamicAABBTreeCollisionManagerd = DynamicAABBTreeCollisionManager;
using BroadPhaseCollisionManagerd = BroadPhaseCollisionManager;
};

#else
// The code has been adapted as if the API were from at least 0.6.0
namespace fcl {
template<typename S>
class CollisionObject;
template<typename S>
class DynamicAABBTreeCollisionManager;
template<typename S>
class BroadPhaseCollisionManager;

using CollisionObjectd = CollisionObject<double>;
using DynamicAABBTreeCollisionManagerd = DynamicAABBTreeCollisionManager<double>;
using BroadPhaseCollisionManagerd = BroadPhaseCollisionManager<double>;
};

#endif

#else

namespace fcl {
template<typename S>
class CollisionObject;
template<typename S>
class DynamicAABBTreeCollisionManager;
template<typename S>
class BroadPhaseCollisionManager;

using CollisionObjectd = CollisionObject<double>;
using DynamicAABBTreeCollisionManagerd = DynamicAABBTreeCollisionManager<double>;
using BroadPhaseCollisionManagerd = BroadPhaseCollisionManager<double>;
};

#endif

namespace rai {

struct FclInterface {
  Array<ptr<struct ConvexGeometryData>> convexGeometryData;
  std::vector<fcl::CollisionObjectd*> objects;
  shared_ptr<fcl::BroadPhaseCollisionManagerd> manager;

  double cutoff=0.; //0 -> perform fine boolean collision check; >0 -> perform fine distance computations; <0 -> only broadphase
  uintA collisions; //return values!
  arr X_lastQuery;  //memory to check whether an object has moved in consecutive queries

  FclInterface(const Array<ptr<Mesh>>& geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X);

private: //called by collision callback
  void addCollision(void* userData1, void* userData2);
  static bool BroadphaseCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* cdata_);
};

}

