#ifndef DYNAMICS_H
#define DYNAMICS_H


#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
//#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
//#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
//#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
//#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <addons/urdfreader/urdfreader.h>
//using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class dynamics
{
public:
    dynamics();
    bool init();
    void inverseDynamics(    const VectorNd &Q,
                             const VectorNd &QDot,
                             const VectorNd &QDDot,
                             VectorNd &Tau,
                             std::vector<SpatialVector> *f_ext = NULL);

    void forwardDynamics (  const VectorNd &Q,
                            const VectorNd &QDot,
                            const VectorNd &Tau,
                            VectorNd &QDDot,
                            std::vector<SpatialVector> *f_ext = NULL);
    bool inverseKinematics(Vector3d pos, Matrix3d ori,VectorNd& qres, int sig);
    bool inverseKinematics(Vector3d pos, Vector3d rpy,VectorNd& qres, int sig);
    void forwardKinematics(VectorNd qin, Vector3d& endpos,Matrix3d& endori);
    void forwardKinematics(VectorNd qin, Vector3d& endpos,Vector3d& endrpy);

    void getEndPose(Vector3d& pos, Matrix3d& ori);
    void getEndPose(Vector3d& pos, Vector3d& rpy);

    void setEndPose(const Vector3d pos, const Matrix3d ori);
    void setEndPose(const Vector3d pos, const Vector3d rpy);
    void getEndPosition(Vector3d& pos);
    void getEndOrientation(Vector3d& rpy);

    void setJointPositions(const VectorNd jp);
    void getJointPositions(VectorNd& jp);

    int getDofs();

    Matrix3d rpyToMatrix(Vector3d rpy);


    void updatalinks();
    void fkgetlinkpose(VectorNd qin, uint link_id, Vector3d& endpos,Matrix3d& endori);
    bool limitate(VectorNd& q, VectorNd& qres);

    int dofs;
    Model* model;
    VectorNd    q;
    VectorNd    qd;
    VectorNd    qdd;
    VectorNd    tau;
    unsigned int end;
    VectorNd jointbuffer;
    Vector3d endpoint;
//    std::vector<Vector3d> linkref={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    std::vector<Vector3d> linkref={{0,0,0},
                                   {0,-0.205,0},
                                   {0,-0.1,0},
                                   {0,0,-0.25},
                                   {0,0,-0.0856},
                                   {0,0,-0.1}
                                    };
    std::vector<Vector3d> linkpos = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    std::vector<Vector3d> linkrpy = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

    std::vector<Matrix3d> linkrpym={Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity()};

    std::vector<Matrix3d> linkposm={Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity(),
                                     Matrix3d::Identity()};





    uint linkids[12];
};

#endif // dynamics_H
