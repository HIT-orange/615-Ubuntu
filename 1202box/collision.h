#ifndef COLLISION_H
#define COLLISION_H
#include <gtest/gtest.h>

#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/continuous_collision.h"

#include "test_fcl_utility.h"

#include "fcl/config.h"

#include "fcl/broadphase/broadphase_collision_manager.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"

#include <unistd.h>


//#include "pcd_io.h"
//#include "point_types.h"


//int num_max_contacts = std::numeric_limits<int>::max();
//bool enable_contact = true;

namespace collisionspace
{
    using namespace std;
    using namespace fcl;




    class collision
    {

    public:
        collision();

        bool addobstacle(char* filename, fcl::Transform3d & addedobstrans);

        bool initrobot();
        bool initenvironment();
        bool addpointcloud();
        bool updatetransforms();

        bool selfcollided();
        bool obscollide();
        bool updateobstacle();
        void fclshow();

        std::vector<Contact<double>>& global_pairs()
        {
          static std::vector<Contact<double>> static_global_pairs;
          return static_global_pairs;
        }


        std::vector<Contact<double>>& global_pairs_now()
        {
          static std::vector<Contact<double>> static_global_pairs_now;
          return static_global_pairs_now;
        }

        typedef BVHModel<OBBRSSd> modeltype;

        bool collided(const Transform3<double>& tf1,
                      const Transform3<double>& tf2,
                      BVHModel<modeltype>& obj1,
                      BVHModel<modeltype>& obj2);


        std::vector<std::shared_ptr<modeltype>> robotgeom = {std::make_shared<modeltype>(),
                                                               std::make_shared<modeltype>(),
                                                               std::make_shared<modeltype>(),
                                                               std::make_shared<modeltype>(),
                                                               std::make_shared<modeltype>(),
                                                               std::make_shared<modeltype>()};


        std::vector<std::shared_ptr<modeltype>> obsgeom={std::make_shared<modeltype>(),
                                                         std::make_shared<modeltype>(),
                                                         std::make_shared<modeltype>(),
                                                         std::make_shared<modeltype>(),
                                                         std::make_shared<modeltype>(),
                                                         std::make_shared<modeltype>(),
                                                                        };



        std::vector<Transform3<double>> obstacletransforms = {Transform3d::Identity(),
                                                                Transform3d::Identity(),
                                                                Transform3d::Identity(),
                                                                Transform3d::Identity(),
                                                                Transform3d::Identity(),
                                                                Transform3d::Identity()};

        std::vector<Transform3<double>> robottransforms={Transform3d::Identity(),
                                                        Transform3d::Identity(),
                                                        Transform3d::Identity(),
                                                        Transform3d::Identity(),
                                                        Transform3d::Identity(),
                                                        Transform3d::Identity()};


        std::vector<CollisionObjectd*> robotobj={};


        std::vector<CollisionObjectd*> obstacleobj={};

        DynamicAABBTreeCollisionManagerd * robotmanager = new DynamicAABBTreeCollisionManagerd();
        DynamicAABBTreeCollisionManagerd * baserobotmanager = new DynamicAABBTreeCollisionManagerd();
        DynamicAABBTreeCollisionManagerd * endrobotmanager = new DynamicAABBTreeCollisionManagerd();

        DynamicAABBTreeCollisionManagerd * obstaclemanager = new DynamicAABBTreeCollisionManagerd();

    };

}





#endif // COLLISION_H
