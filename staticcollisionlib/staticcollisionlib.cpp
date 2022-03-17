#include "staticcollisionlib.h"

Staticcollisionlib::Staticcollisionlib()
{
}
fcl::CollisionObjectd * Staticcollisionlib::addobj(char *  path)
{
    std::vector<fcl::Vector3d> p;
    std::vector<fcl::Triangle> t;
    std::shared_ptr<modeltype> obsgeom = std::make_shared<modeltype>();

    fcl::Transform3<double> obstacletransforms = fcl::Transform3d::Identity();

    fcl::test::loadOBJFile(path,p,t);
    obsgeom->bv_splitter.reset(new fcl::detail::BVSplitter<fcl::OBBRSS<double>>(fcl::detail::SPLIT_METHOD_MEAN));
    obsgeom->beginModel();
    obsgeom->addSubModel(p, t);
    obsgeom->endModel();
    cout << "obstacle 0 vertices: "  << obsgeom->num_vertices << endl;
    cout << "obstacle 0 triangles:"  << obsgeom->num_tris << endl;

    fcl::CollisionObjectd* obj;
    obj = new fcl::CollisionObjectd(obsgeom,obstacletransforms);


    return obj;

}

bool Staticcollisionlib::checkcollision(fcl::CollisionObjectd *obj1,fcl::CollisionObjectd *obj2)
{
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(obj1,obj2,request,result);
    return result.isCollision();

}
