#ifndef STATICCOLLISIONLIB_H
#define STATICCOLLISIONLIB_H

#include <gtest/gtest.h>
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcllocal/test/test_fcl_utility.h"
#include "fcl/config.h"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include <unistd.h>

using namespace  std;

 typedef fcl::BVHModel<fcl::OBBRSSd> modeltype;

class Staticcollisionlib
{
public:
    Staticcollisionlib();



    fcl::CollisionObjectd * addobj(char * path);

    bool checkcollision(fcl::CollisionObjectd *obj1,fcl::CollisionObjectd *obj2);






    std::vector<fcl::Contact<double>>& global_pairs()
    {
      static std::vector<fcl::Contact<double>> static_global_pairs;
      return static_global_pairs;
    }


    std::vector<fcl::Contact<double>>& global_pairs_now()
    {
      static std::vector<fcl::Contact<double>> static_global_pairs_now;
      return static_global_pairs_now;
    }








};

#endif // STATICCOLLISIONLIB_H
