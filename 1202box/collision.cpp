#include "collision.h"
using namespace collisionspace;

collision::collision()
{

}



bool collision::initenvironment()
{
    std::vector<Vector3<double>> p;
    std::vector<Triangle> t;

    test::loadOBJFile("../../meshes/obstacle2.obj", p, t);
//    test::loadOBJFile("../../meshes/longbox.obj",p,t); //test import model's original position
    obsgeom[0]->bv_splitter.reset(new detail::BVSplitter<OBBRSS<double>>(detail::SPLIT_METHOD_MEAN));
    obsgeom[0]->beginModel();
    obsgeom[0]->addSubModel(p, t);
    obsgeom[0]->endModel();
    cout << "obstacle 0 vertices: "  << obsgeom[0]->num_vertices << endl;
    cout << "obstacle 0 triangles:"  << obsgeom[0]->num_tris << endl;

    obstacleobj.push_back(new CollisionObjectd(obsgeom[0],obstacletransforms[0]));
    cout << "obstacle transforms : " << obstacletransforms[0].linear() << endl<< obstacletransforms[0].translation() << endl;
    obstaclemanager->registerObject(obstacleobj[0]);
    obstaclemanager->setup();
    cout << " obstacle 0 init succeed " << endl;
    return true;
}

bool collision::addpointcloud()
{


    return true;
}
//bool collision::obscollide()
//{
//    /*return true if collided*/
//    cout << "obstacle collision ....." << endl;

//        //for test!!! check every pair
//        bool result[6];
//        result[0] = collided(transforms,transforms1,obstacles,link1);
//        result[1] = collided(transforms,transforms2,obstacles,link2);
//        result[2] = collided(transforms,transforms3,obstacles,link3);
//        result[3] = collided(transforms,transforms4,obstacles,link4);
//        result[4] = collided(transforms,transforms5,obstacles,link5);
//        result[5] = collided(transforms,transforms6,obstacles,link6);

//        for(int i = 0;i<6;i++)
//        {
//            cout << "collision between link " << i+1 << " and obstacle: " << result[i]<< endl;
//        }
//        if(result[0]||result[1]||result[2]||result[3]||result[4]||result[5]) return true;

//        //for using !!! once collided, break out
//        if(     collided(transforms,transforms1,obstacles,link1)
//                ||collided(transforms,transforms2,obstacles,link2)
//                ||collided(transforms,transforms3,obstacles,link3)
//                ||collided(transforms,transforms4,obstacles,link4)
//                ||collided(transforms,transforms5,obstacles,link5)
//                ||collided(transforms,transforms6,obstacles,link6))
//        {
//         return true;
//        }

//    return false;
//}


//bool collision::collided(const Transform3<double>& tf1,const Transform3<double>& tf2,
//                  BVHModel<OBB<double>>& mesh1,BVHModel<OBB<double>>& mesh2)
//{
////    /*return true if collided*/
//    //erase last collide check information???
////    global_pairs().clear();
////    global_pairs_now().clear();
//    //create new models with the same mesh of imported links or obstacles !!!!!!!!!!
//    BVHModel<OBB<double>> obj1(mesh1);
//    BVHModel<OBB<double>> obj2(mesh2);

//    Transform3<double> pose1(tf1);
//    Transform3<double> pose2(tf2);
//    cout << "********************  colliding pair positions: " << endl << pose1.translation()<< endl<< pose2.translation() << endl;
//    cout << "********************  colliding pair rotation : " << endl << pose1.linear()<< endl<< pose2.linear() << endl;

//    cout << " mesh 1 volume : " << obj1.computeVolume()<< endl;
//    cout << " mesh 2 volume : " << obj2.computeVolume()<< endl;

//    cout << "object 1 center :" << obj1.computeCOM() << endl;
//    cout << "object 2 center :" << obj2.computeCOM() << endl;


//    CollisionResult<double> local_result;

//    detail::MeshCollisionTraversalNode<OBB<double>> node;

//    CollisionRequest<double> collision_request(std::numeric_limits<int>::max(), true,10,true);
//    if(!detail::initialize<OBB<double>>(node, obj1, pose1, obj2, pose2,
//             collision_request, local_result))
//    {
////        std::cout << "initialize error" << std::endl;
//    }
//    else
//    {
//        std::cout<< "initialize succeeded" << std::endl;
//        std::cout << " solver type : " << collision_request.gjk_solver_type << endl;
//    }

//    node.enableStatistics(false);

//    collide(&node);


//    cout << " collision resutl : " << local_result.isCollision() << endl;

//    if(local_result.numContacts() > 0)
//    {
//        if(global_pairs().size() == 0)
//        {
//            local_result.getContacts(global_pairs());
//            std::sort(global_pairs().begin(), global_pairs().end());
//        }
//        else
//        {
//            local_result.getContacts(global_pairs_now());
//            std::sort(global_pairs_now().begin(), global_pairs_now().end());
//        }
//        std::cout << "____________________________Collided... " << local_result.numContacts() << ": " << std::endl;
//        std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;

//        return true;
//    }
//    else
//    {
//        std::cout << "_____________________________Collision free " << std::endl;
//        std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
//        return false;
//    }
//}
//bool collision::selfcollided()
//{
//    cout<<"self collision ...... " << endl;
//    if(       collided(transforms1,transforms3,link1,link3)
//            ||collided(transforms1,transforms4,link1,link4)
//            ||collided(transforms1,transforms5,link1,link5)
//            ||collided(transforms1,transforms6,link1,link6)
//            ||collided(transforms2,transforms4,link2,link4)
//            ||collided(transforms2,transforms5,link2,link5)
//            ||collided(transforms2,transforms6,link2,link6))
//    {
//        return true;
//    }
//    else return false;
//}


