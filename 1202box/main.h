#ifndef MAIN_H
#define MAIN_H

#include "OmplSpace.h"
#include "vrepshow.h"
#include "dynamics.h"
#include "collision.h"
#include "myplanner.h"
#include <algorithm>
#include "cubicpolynomial.h"
#include "motionplanning.h"

const float T_SIM = 0.01;
motionplanning m_traj;

using namespace std;
namespace vp = vrepspace;
namespace rbdlmath = RigidBodyDynamics::Math;
namespace cs = collisionspace;
polynomial m_cubic;
cs::collision m_collision;
vp::VrepShow m_vrep;
dynamics m_dyn;


/*******************************        fcl        **************************************************************************/
bool collide();
bool obstaclemoved();
//bool cs::collision::updatetransforms();
//bool cs::collision::updateobstacle();
//bool cs::collision::initrobot();
//void cs::collision::fclshow();

/*******************************        ompl        **************************************************************************/

//void OmplStateSpace::ShowState(const ob::ScopedState<OmplStateSpace>& s);
//bool myStateValidityCheckerClass::isValid(const ob::State *state) const;
//bool configkinovaompl(og::SimpleSetup& setup,ob::SpaceInformationPtr& si, rbdlmath::VectorNd & startposdeg, rbdlmath::VectorNd & goalposdeg)


/*******************************        rbdl        **************************************************************************/

void vrep_setrbdllinks();
rbdlmath::VectorNd  validik(rbdlmath::Vector3d & endposv,rbdlmath::Matrix3d & endorim, int sig);

/*******************************        transform       **************************************************************************/

rbdlmath::VectorNd radtoang(rbdlmath::VectorNd &jointrads);
rbdlmath::VectorNd angtorad(rbdlmath::VectorNd &jointangles);
void fcltranstovreppose(const cs::Transform3<double>& tf1,vp::pos3d & pose,float *vreprpy);
void floattovector(float* floats, VectorNd &vectors);
void vectortofloat(VectorNd vectors, float* floats);

/*******************************        vrep       **************************************************************************/

void vrep_setjoints(rbdlmath::VectorNd & jointrad);
bool vrep_setpos(int handle,  vector<double>& pose, vector<double>& ori, int refhandle);
bool vrep_setpose(int link_id,Vector3d &pos,Vector3d & ori,int refhandle = -1);
bool vrep_setpose(int link_id,Vector3d &pos,Matrix3d & mat,int refhandle = -1);

/*******************************        main       **************************************************************************/
void initall();
void setstart(rbdlmath::VectorNd &startposrad );
void setgoal(rbdlmath::Vector3d& endposv,rbdlmath::VectorNd& goalposrad);
bool showpath(og::SimpleSetup& setup, int stage);
void replan();



void initall()
{
    //1. initialize rbdl model for kinematics first
    m_dyn.init();

    //2. initialize vrep show secondly
    m_vrep.vrep_connect("192.168.15.4");
    m_vrep.vrep_start();

    //3. initialize robot mesh for fcl collision check
    cout << "initialize environment mesh " << endl;
    m_collision.initenvironment();

    cout << "initialize robot mesh " << endl;
    m_collision.initrobot();

    /*set moveable obstacle transforms*/
    fcl::Vector3d obstacleposition = {0,0.54,0.3};
    fcl::Vector3d obstacleori = {0,0,M_PI/2};
    fcl::Matrix3d obstacleorientation = m_dyn.rpyToMatrix(obstacleori);
    m_collision.obstacletransforms[0].linear() = obstacleorientation;
    m_collision.obstacletransforms[0].translation() = obstacleposition;
    m_collision.updateobstacle();

    //test add obstacles
    fcl::Transform3d addedobstrans = fcl::Transform3d::Identity();
    fcl::Vector3d addedobspos = {0,0,-0.1};
    fcl::Vector3d addedobsrpy = {0,0,0};


//    char filename[64] = "chair.obj";
//    m_collision.addobstacle(filename,addedobstrans);

    addedobspos ={-0.45,0.0,0.06};
    char rack[64] = "1202box";
    addedobstrans.translation() = addedobspos;
    m_collision.addobstacle(rack,addedobstrans);

    addedobspos ={-0.8,0.0,0.2};
    char obs2[64] = "obstacle2";
    addedobstrans.translation() = addedobspos;
    m_collision.addobstacle(obs2,addedobstrans);

    char floor[64] = "floor";
    addedobspos ={0,0.0,-0.1};
    addedobstrans.translation() = addedobspos;
    m_collision.addobstacle(floor,addedobstrans);

    //init random seed
    srand(time(NULL));
}

bool cs::collision::addobstacle(char* filename, fcl::Transform3d &addedobstrans)
{
    std::vector<Vector3<double>> p1;
    std::vector<Triangle> t1;

    char files[64];
    sprintf(files, "../../meshes/%s.obj",filename);
    cout << "----------------------------------------------   sprintf : " << files << endl;



    test::loadOBJFile(files, p1, t1);
    int index = obstaclemanager->size();
    cout << "current obstacle number : " <<index << endl;

    obsgeom.push_back(std::make_shared<modeltype>());
    obsgeom[index]->bv_splitter.reset(new detail::BVSplitter<OBBRSS<double>>(detail::SPLIT_METHOD_MEAN));
    obsgeom[index]->beginModel();
    obsgeom[index]->addSubModel(p1,t1);
    obsgeom[index]->endModel();

    simxGetObjectHandle(m_vrep.client_id,filename,&m_vrep.obstacle_handle[index],simx_opmode_blocking);



    cout << "obstacle 0 vertices: "  << obsgeom[index]->num_vertices << endl;
    cout << "obstacle 0 triangles:"  << obsgeom[index]->num_tris << endl;
    cout << "obstacle "<<index << " transforms : " << obstacletransforms[index].linear() << endl<< obstacletransforms[index].translation() << endl;

    obstacleobj.push_back(new CollisionObjectd(obsgeom[index],obstacletransforms[index]));
    obstaclemanager->registerObject(obstacleobj[index]);
    obstaclemanager->setup();

    obstacletransforms[index].translation() = addedobstrans.translation();

    cout << "obstacle " << index << " center is : " << obsgeom[index]->computeCOM() << endl;
    cout << "obstacle number after add : " << obstaclemanager->size() << endl;

}
void endsetstart(rbdlmath::VectorNd &startposrad )
{

}


void setstart(rbdlmath::VectorNd &startposrad )
{
    for(int i = 0;i<6;i++)
    {
        m_dyn.q[i] =startposrad[i];
        cout << " q[ " << i << "] is : " << m_dyn.q[i] << endl;
    }
//    m_vrep.vrep_setjoints();

    //update start pos
    m_dyn.updatalinks();
    m_collision.updatetransforms();

    m_collision.fclshow();

    /*start position collision test*/
    cout<<"************************   start position collided ?  *******************  " << endl;
//    for(int i =0;i<6;i++)
//    {
//        cs::DefaultCollisionData<double> cd1;
//        cs::DefaultCollisionData<double> cd2;
//        cs::CollisionRequestd request2;
//        cs::CollisionResultd result2;


//        m_collision.obstaclemanager->collide(m_collision.robotobj[i],&cd1,cs::DefaultCollisionFunction);
//        m_collision.robotmanager->collide(m_collision.obstacleobj[1],&cd2,cs::DefaultCollisionFunction);
//        cout << " obstacle manager collide with robot link : " << i+1 << "  result : " << cd1.result.isCollision() << endl;
//        cout << " robot manager collide with obstacle                       result : " << cd2.result.isCollision() << endl;

//        cs::collide(m_collision.obstacleobj[1],m_collision.robotobj[i],request2,result2);
//        cout << " obstacle 1 collide with robot link " << i+1 << " result : " <<result2.isCollision()<< endl;
//    }
//    cs::CollisionResultd result;
//    cs::CollisionRequestd request;
//    cs::collide(m_collision.obstacleobj[0],m_collision.obstacleobj[1],request,result);
//    cout << "obstacle 0 collide with obstacle 1 ? : " << result.isCollision() << endl;

    int collideresult = collide();
    cout << "start position collision test : " << collideresult << endl;
}

void replan()
{

}


bool showendpath(og::SimpleSetup& setup, int stage)
{
    cout << "---------------- TIME USED : " << setup.getLastPlanComputationTime() << " ---------------" << endl;

    if (!setup.haveSolutionPath())
    {
        cout << "......  NO SOLUTION PATH FOUNDED  ......" << endl;
        return false;
    }

    if (setup.haveExactSolutionPath())
    {
        cout << "......  EXACT SOLUTION FOUNDED  ...... " << endl;
        int length;
        ob::ScopedState<OmplStateSpace> pathstate(setup.getStateSpace());

        switch (stage)
        {
        case 1:
        {
            /*vrep show sampling process (do state validity check during this period)*/
            cout<< "********************* show sampling process **********************"<<endl;
            length = m_vrep.jointbuffer.size();
            cout<< "rrtstar exploring tree nodes number: " << (length+1)/6 << endl;
            double jointangles[6] = {0,0,0,0,0,0};
            for(int i = 0; i < length; i=i+6)
            {
                cout << " up date states : " << (i+6)/6 << endl;
                for(int j = 0;j<6;j++)
                {
                    jointangles[j]=m_vrep.jointbuffer[i+j];
                }

                 m_vrep.vrep_setjoints(jointangles);
            }
            break;
        }
        case 2:
        {
            //// /*show init planned path with fewer path points*/
            cout << "---------------     show planned path points     -----------------" << endl;
            length = setup.getSolutionPath().getStateCount();
            cout << " initial path length is : " << length << endl;
            for (int i = 0; i < length; i++)
            {
                cout<< "*******************  path point : " << i <<"  *******************"<<endl;
                pathstate = setup.getSolutionPath().getState(i);
//                pathstate.print(cout);
                setup.getStateSpace()->as<OmplStateSpace>()->ShowState(pathstate);
                extApi_sleepMs(2000);
            }
            break;
        }
        case 3:
        {
            /*show planned motion in vrep (after state validity check)*/
            cout<< "******************    show interpolated path    *******************"<<endl;

//            setup.simplifySolution();
            setup.getSolutionPath().interpolate(100);
            length = setup.getSolutionPath().getStateCount();
            int len = setup.getSolutionPath().length();
            cout << " interpolated path point number is : " << length << endl;
            cout << " interpolated path length is : " << len << endl;
            for (int i = 0; i < length; i++)
            {
//                cout<< "*******************  path point : " << i <<"  ********************"<<endl;
                pathstate = setup.getSolutionPath().getState(i);
                pathstate.print(cout);

                /*show joint space planning solution in vrep */
                setup.getStateSpace()->as<endStateSpace>()->ShowState(pathstate);
            }
            break;
        }
        default: break;
        }
        return true;
    }
    else
    {
        cout << "NO EXACT SOLUTION PATH FOUNDED" << endl;
        return false;
    }
}


bool showpath(og::SimpleSetup& setup, int stage)
{
    cout << "---------------- TIME USED : " << setup.getLastPlanComputationTime() << " ---------------" << endl;

    if (!setup.haveSolutionPath())
    {
        cout << "......  NO SOLUTION PATH FOUNDED  ......" << endl;
        return false;
    }

    if (setup.haveExactSolutionPath())
    {
        cout << "......  EXACT SOLUTION FOUNDED  ...... " << endl;
        int length;
        ob::ScopedState<OmplStateSpace> pathstate(setup.getStateSpace());

        switch (stage)
        {
        case 1:
        {
            /*vrep show sampling process (do state validity check during this period)*/
            cout<< "********************* show sampling process **********************"<<endl;
            length = m_vrep.jointbuffer.size();
            cout<< "rrtstar exploring tree nodes number: " << (length+1)/6 << endl;
            double jointangles[6] = {0,0,0,0,0,0};
            for(int i = 0; i < length; i=i+6)
            {
                cout << " up date states : " << (i+6)/6 << endl;
                for(int j = 0;j<6;j++)
                {
                    jointangles[j]=m_vrep.jointbuffer[i+j];
//                    extApi_sleepMs(100);
                }

                 m_vrep.vrep_setjoints(jointangles);
            }
            break;
        }
        case 2:
        {
            //// /*show init planned path with fewer path points*/
            cout << "---------------     show planned path points     -----------------" << endl;
            length = setup.getSolutionPath().getStateCount();
            cout << " initial path length is : " << length << endl;
            for (int i = 0; i < length; i++)
            {
                cout<< "*******************  path point : " << i <<"  *******************"<<endl;
                pathstate = setup.getSolutionPath().getState(i);
//                pathstate.print(cout);
                setup.getStateSpace()->as<OmplStateSpace>()->ShowState(pathstate);
                extApi_sleepMs(2000);
            }
            break;
        }
        case 3:
        {
            /*show planned motion in vrep (after state validity check)*/
            cout<< "******************    show interpolated path    *******************"<<endl;

//            setup.simplifySolution();
            setup.getSolutionPath().interpolate(500);
            length = setup.getSolutionPath().getStateCount();
            int len = setup.getSolutionPath().length();
            cout << " interpolated path point number is : " << length << endl;
            cout << " interpolated path length is : " << len << endl;
            for (int i = 0; i < length; i++)
            {
//                cout<< "*******************  path point : " << i <<"  ********************"<<endl;
                pathstate = setup.getSolutionPath().getState(i);
                pathstate.print(cout);

                /*show joint space planning solution in vrep */
                setup.getStateSpace()->as<OmplStateSpace>()->ShowState(pathstate);
            }
            break;
        }
        default: break;
        }
        return true;
    }
    else
    {
        cout << "NO EXACT SOLUTION PATH FOUNDED" << endl;
        return false;
    }
}

void setgoal(rbdlmath::Vector3d& endposv,rbdlmath::VectorNd& goalposrad)
{
    //set vrep goal dummy
    vp::pos3d pose;
    pose.posx=endposv[0];
    pose.posy=endposv[1];
    pose.posz=endposv[2];
    float fakerpy[3]={0,0,0};
    m_vrep.vrep_setpose3d(m_vrep.goal_handle,-1,pose,fakerpy);

    vrep_setjoints(goalposrad);
    m_dyn.setJointPositions(goalposrad);
    m_dyn.updatalinks();
    m_collision.updatetransforms();
    m_collision.fclshow();

    /*goal position collision test*/
    cout<<"************************   goal position collided ?  *******************  " << endl;
//    for(int i =0;i<6;i++)
//    {
//        cs::DefaultCollisionData<double> cd1;
//        cs::DefaultCollisionData<double> cd2;
//        cs::CollisionRequestd request2;
//        cs::CollisionResultd result2;


//        m_collision.obstaclemanager->collide(m_collision.robotobj[i],&cd1,cs::DefaultCollisionFunction);
//        m_collision.robotmanager->collide(m_collision.obstacleobj[1],&cd2,cs::DefaultCollisionFunction);
//        cout << " obstacle manager collide with robot link : " << i+1 << "  result : " << cd1.result.isCollision() << endl;
//        cout << " robot manager collide with obstacle                       result : " << cd2.result.isCollision() << endl;

//        cd1.result.clear();
//        cs::collide(m_collision.obstacleobj[1],m_collision.robotobj[i],request2,result2);
//        cout << " obstacle 1 collide with robot link " << i+1 << " result : " <<result2.isCollision()<< endl;
//    }
//    cs::CollisionResultd result;
//    cs::CollisionRequestd request;
//    cs::collide(m_collision.obstacleobj[1],m_collision.obstacleobj[0],request,result);
//    cout << "obstacle 0 collide with obstacle 1 ? : " << result.isCollision() << endl;

    int collisionresult = collide();
    cout << "goal position collision test : " << collisionresult << endl;

}

rbdlmath::VectorNd radtoang(rbdlmath::VectorNd &jointrads)
{
    rbdlmath::VectorNd jointangles(6);
    for(int i = 0; i<6;i++)
    {
        jointangles[i] = jointrads[i]*180/M_PI;
    }
    return jointangles;
}

rbdlmath::VectorNd angtorad(rbdlmath::VectorNd &jointangles)
{
    rbdlmath::VectorNd jointrads(6);
    for(int i = 0; i<6;i++)
    {
        jointrads[i] = jointangles[i]*M_PI/180;
    }
    return jointrads;
}

void fcltranstovreppose(const cs::Transform3<double>& tf1,vp::pos3d & pose,float *vreprpy)
{
    cs::Transform3<double> transform(tf1);
    fcl::Vector3d T = transform.translation();
    pose.posx = T[0];
    pose.posy = T[1];
    pose.posz = T[2];
//    cout<<"transforms.translation: " <<endl << T[0] <<endl << T[1] <<endl << T[2]<< endl;
//    cout << "pos3d  : " << pose.posx <<endl<< pose.posy <<endl<< pose.posz << endl;
    fcl::Matrix3d R = transform.rotation();
//    cout <<"transform.rotation: " <<endl<< R<< endl;
    fcl::Vector3d mid = R.eulerAngles(0,1,2);
    vreprpy[0] = mid[0];
    vreprpy[1] = mid[1];
    vreprpy[2] = mid[2];
}
void endStateSpace::ShowState(const ob::ScopedState<endStateSpace>& s)
{
    simxFloat pos[3];
    Vector3d endposv={0,0,0};
    Matrix3d fakeorim = Matrix3d::Identity();
    Vector4d quan;
    for(int i =0;i<3;i++)
    {
        pos[i] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
        endposv[i] = pos[i];
    }

//    quan[0] = s->as<ob::SO3StateSpace::StateType>(3)->w;
//    quan[1] = s->as<ob::SO3StateSpace::StateType>(3)->x;
//    quan[2] = s->as<ob::SO3StateSpace::StateType>(3)->y;
//    quan[3] = s->as<ob::SO3StateSpace::StateType>(3)->z;

    VectorNd joints = validik(endposv,fakeorim,1);
    m_dyn.setJointPositions(joints);
    m_dyn.updatalinks();
    m_collision.updatetransforms();
    m_collision.fclshow();

    vrep_setjoints(joints);
    simxSetObjectPosition(m_vrep.client_id,m_vrep.goal_handle,-1,pos,simx_opmode_oneshot);
    extApi_sleepMs(10);
}
void OmplStateSpace::ShowState(const ob::ScopedState<OmplStateSpace>& s)
{
    simxFloat jointposition;
    VectorNd jointd(6);
    VectorNd jointabgd(6);
    for (int i = 0; i < 6; i++)
    {
        //set ompl sampled joint points
        jointposition = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
        jointposition = (jointposition * M_PI) / 180;
        //ompl sampled joint space points
        simxSetJointPosition(m_vrep.client_id, m_vrep.joint_handle[i], jointposition, simx_opmode_oneshot);
        //set fcl links straightly
//        jointabgd[i] = (double)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
    }
    extApi_sleepMs(10);

//    angtorad(jointabgd,jointd);
//    m_dyn.setJointPositions(jointd);
//    m_dyn.updatalinks();
//    m_collision.updatecollisionobjects();
//    m_collision.fclshow();

}

bool myStateValidityCheckerClass::isValid(const ob::State *state) const
{
    /*return the state validity: false(collided) or true(no collision)   (collide with obstacles or itself)*/
    VectorNd jp(6);
    bool res = false;
    //sample joint space points:
    for (int i = 0; i < 6; i++)
    {
        jp[i] = (float)(state->as<OmplStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(i)->values[0])*M_PI/180;
        //simxSetJointPosition(m_vrep.client_id, m_vrep.joint_handle[i], jp[i], simx_opmode_oneshot);
        m_vrep.jointbuffer.push_back(jp[i]);//used for show sampling process...
    }
    VectorNd lastjp;
    m_dyn.getJointPositions(lastjp);
    m_dyn.setJointPositions(jp);
    //update forward kinematics :
    m_dyn.updatalinks();
    //update collision object pose:
    m_collision.updatetransforms();


    res = collide(); //return true if collided

//    cout << "validity check ans: " << res << endl;

    if (res==true)
    {
        m_dyn.setJointPositions(lastjp);
        return false;
    }
    else return true;
//  /*return the state validity: false(collided) or true(no collision)   (collide with obstacles or itself)*/
}
bool endStateValidityCheckerClass::isValid(const ob::State *state) const
{
    bool result = false;

    simxFloat pos[3];
    Vector3d endposv={0,0,0};
    Matrix3d fakeorim = Matrix3d::Identity();
    Vector4d quan;
    VectorNd joints(6);
    for(int i =0;i<3;i++)
    {
        pos[i] = (float)(state->as<endStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(i)->values[0]);
        endposv[i] = pos[i];
    }

//    quan[0] = s->as<ob::SO3StateSpace::StateType>(3)->w;
//    quan[1] = s->as<ob::SO3StateSpace::StateType>(3)->x;
//    quan[2] = s->as<ob::SO3StateSpace::StateType>(3)->y;
//    quan[3] = s->as<ob::SO3StateSpace::StateType>(3)->z;

    /*check all links collision*/
    bool res = m_dyn.inverseKinematics(endposv,fakeorim,joints,1);
    if (res==false) return false;// no ik solutin for this end point
    m_dyn.setJointPositions(joints);
    m_dyn.updatalinks();
    m_collision.updatetransforms();
    result = collide();

    /*check link6 collision*/
    cs::DefaultCollisionData<double> cdata;
    m_collision.robotobj[5]->setTranslation(endposv);
    m_collision.obstaclemanager->collide(m_collision.robotobj[5],&cdata,cs::DefaultCollisionFunction);
    result = cdata.result.isCollision();



    if (result==true)
        return false;
    else return true;
}
bool cs::collision::updatetransforms()
{
    /*updata robot and obstacle transforms and update collision managers */

    // robot part:
    for(int i=0;i<6;i++)
    {
        robottransforms[i].linear() = m_dyn.linkrpym[i];
        robottransforms[i].translation() = m_dyn.linkpos[i];
    }


    for(int i=0;i<6;i++)
    {
        robotobj[i]->setRotation(m_dyn.linkrpym[i]);
        robotobj[i]->setTranslation(m_dyn.linkpos[i]);
//        cout << "update robot link " <<i+1<< " transforms..." << endl;
//        std::cout << "robottransforms : " << endl<< robottransforms[i].linear()<< std::endl << robottransforms[i].translation() << std::endl;
        robotmanager->update(robotobj[i]);

    }

    baserobotmanager->update(robotobj[0]);
    baserobotmanager->update(robotobj[1]);
//    baserobotmanager->update(robotobj[2]);

    endrobotmanager->update(robotobj[3]);
    endrobotmanager->update(robotobj[4]);
    endrobotmanager->update(robotobj[5]);


    //obstacle part:
    int size = obstaclemanager->size();
//    cout << "update transforms for obstacle num: " << size << endl;
    for (int i =0;i<size;i++)
    {             
        obstacleobj[i]->setTransform(m_collision.obstacletransforms[i].linear(),m_collision.obstacletransforms[i].translation());
//        cout << "update obstacle    " << i << "    transforms "<< endl;
//        cout <<m_collision.obstacletransforms[i].linear() << endl << m_collision.obstacletransforms[i].translation() << endl;
        obstaclemanager->update(obstacleobj[i]);
//        cout << "obstacle updated volume : " << obstacleobj[i]->collisionGeometry()->computeVolume() << endl;
    }


    //show in vrep:
//    fclshow();
    return true;
}

bool cs::collision::updateobstacle()
{
    int size = obstaclemanager->size();
    for (int i =0;i<size;i++)
    {
        obstacleobj[i]->setTransform(m_collision.obstacletransforms[i].linear(),m_collision.obstacletransforms[i].translation());
    }
    obstaclemanager->update(obstacleobj);
    return true;
}

bool cs::collision::initrobot()
{
    std::vector<Vector3<double>> p1;
    std::vector<Triangle> t1;

    for(int i =0;i<6;i++)
    {
     char files[64];
     sprintf(files, "../../meshes/link%d.obj", i + 1);
     test::loadOBJFile(files, p1, t1);

     robotgeom[i]->bv_splitter.reset(new detail::BVSplitter<OBBRSS<double>>(detail::SPLIT_METHOD_MEAN));
     robotgeom[i]->beginModel();
     robotgeom[i]->addSubModel(p1, t1);
     robotgeom[i]->endModel();
    //         cout << "link "<<i+1<<" vertices: "  << robotgeom[i]->num_tris << endl;
    //         cout << "link "<<i+1<<" triangles:"  << robotgeom[i]->num_vertices << endl;
    //         std::cout << "robottransforms : " << endl<< robottransforms[i].linear()<< std::endl << robottransforms[i].translation() << std::endl;

     robotobj.push_back(new cs::CollisionObjectd(robotgeom[i],robottransforms[i]));

//     cout << "robot " << i+1 << " is imported " << endl << endl<<endl;
     p1.clear();
     t1.clear();
    }

    //robot manager will collide with obstacle manager
    for(int i = 0;i<6;i++)
    {
        robotmanager->registerObject(robotobj[i]);
//        cout << "robot manager has link position : " << robotobj[i]->collisionGeometry()->computeCOM()<<endl;
    }

//    robotmanager->registerObjects(robotobj);
//    //delete robot link1 from robotmanager so it doesn't collide with floor
//    robotmanager->unregisterObject(robotobj[0]);

    robotmanager->setup();

    //base and end robot manager will collide with each other as robot self collision check
    for(int i =0;i<2;i++)
    {
        baserobotmanager->registerObject(robotobj[i]);
    }
    baserobotmanager->setup();

    for(int j =3;j<6;j++)
    {
        endrobotmanager->registerObject(robotobj[j]);
    }
    endrobotmanager->setup();


//    fclshow();

    return true;
}

bool collide()
{
    cs::DefaultCollisionData<double> cd1;
    cs::DefaultCollisionData<double> cd2;


//    cout << "collide between robotmanager and obstaclemanager .............." << endl;

//    cout << "robot part: " << m_collision.robotmanager->size() << endl;
//    cout << "obstacle parts: " << m_collision.obstaclemanager->size()<< endl;


    m_collision.robotmanager->collide(m_collision.obstaclemanager,&cd1,cs::DefaultCollisionFunction);
//    cout<<"collision manager: result : " << cd1.result.isCollision() << endl;

    m_collision.baserobotmanager->collide(m_collision.endrobotmanager, &cd2,cs::DefaultCollisionFunction);
//    cout<<"robot self collision result: " << cd2.result.isCollision() << endl;

    //return true if either is collided
    if(cd1.result.isCollision()||cd2.result.isCollision()) return true;

    return false;
}

void cs::collision::fclshow()
{
    vp::pos3d posi;
    float vreprpy[3];
    for(int i =0;i<6;i++)
    {
        fcltranstovreppose(robottransforms[i],posi,vreprpy);
        m_vrep.vrep_setpose3d(m_vrep.link_handle[i],-1,posi,vreprpy);
    }
    int len = obstaclemanager->size();
    for(int i=0;i<len;i++)
    {
        fcltranstovreppose(obstacletransforms[i],posi,vreprpy);
        cout << " set obstacle pos in vrep : " << endl<<
                posi.posx<<endl<<posi.posy<<endl<<posi.posz << endl <<
                "set obstacle ori in vrep :" << endl
             <<vreprpy[0] << endl<<vreprpy[1] << endl<<vreprpy[2] << endl;
        m_vrep.vrep_setpose3d(m_vrep.obstacle_handle[i],-1,posi,vreprpy);
    }

}
ob::PlannerPtr configplanner(ob::SpaceInformationPtr & si)
{
    og::RRTConnect *myrrt = new og::RRTConnect(si);
    myrrt->setRange(600);
    myrrt->setName("rrt-modified");

//    myPlanner *endrrt = new myPlanner(si,"endrrt");
//    endrrt->setRange(1);



    return ob::PlannerPtr(myrrt);
}
bool configkinovaompl(og::SimpleSetup& setup,ob::SpaceInformationPtr& si, rbdlmath::VectorNd & startposrad, rbdlmath::VectorNd & goalposrad)
{

    rbdlmath::VectorNd startposdeg = radtoang(startposrad);
    rbdlmath::VectorNd goalposdeg = radtoang(goalposrad);

    //problem definition
    ob::ProblemDefinitionPtr pdp(setup.getProblemDefinition());

    //set init state of joints
    ob::ScopedState<OmplStateSpace> startstate(si);
    for (int n = 0; n < 6; n++)
    {
        startstate->as<ob::RealVectorStateSpace::StateType>(n)->values[0] = startposdeg[n];
    }

    //set goal
    ob::ScopedState<OmplStateSpace> goalstate(si);
    for (int n = 0; n < 6; n++)
    {
        goalstate->as<ob::RealVectorStateSpace::StateType>(n)->values[0] = goalposdeg[n];
    }

    //set start and goal joints state
    pdp->setStartAndGoalStates(startstate,goalstate);

    //set optimization object
//    ob::OptimizationObjectivePtr opter(pdp->getOptimizationObjective());
//    ob::Cost con(100.0);
//    opter->setCostThreshold(con);


    //set state valid chacker and parameters
    si->setStateValidityChecker(std::make_shared<myStateValidityCheckerClass>(si));
    si->setStateValidityCheckingResolution(0.03);


    //create my own sampler test !
//    ompl::base::ValidStateSamplerPtr samplevalidstate(std::make_shared<mySampler>(si));
//    samplevalidstate->setName("my random sampler");
//    samplevalidstate->setNrAttempts(100);

    cout << "-----------------  planner  ---------------" << endl;
    //using rrtconnnect algorithm
//    setup.setPlanner(std::make_shared<og::RRTConnect>(si));
    setup.setPlanner(configplanner(si));

//    setup.getPlanner()->printProperties(std::cout);

    cout << "------------------simple setup information  ------------------" << endl;
//    setup.print(cout);

    return true;
}

void vectortofloat(VectorNd &vectors, float* floats)
{
    for(int i =0;i<6;i++)
    {
        floats[i] = vectors[i];
    }

}

void floattovector(float* floats, VectorNd &vectors)
{

    for(int i =0;i<6;i++)
    {
        vectors[i] = floats[i];
//        cout << "float j: " << i << " : " << floats[i]<< endl;
//        cout << "goalpos j: " << i << " : " << vectors[i]<< endl;
    }
}

bool vrep_setpos(int handle,  vector<double>& pose, vector<double>& ori, int refhandle)
{
    bool res,ret = 0;
    float position[3] = { (float)pose[0], (float)pose[1], (float)pose[2] };
    float orientation[3] = {(float)ori[0],(float)ori[1],(float)ori[2]} ;


//    cout << "vrep set link[ " << handle<< " ]: ori " << endl << ori[0]*180/M_PI << endl << ori[1]*180/M_PI << endl << ori[2]*180/M_PI  << endl;
    res = simxSetObjectPosition(m_vrep.client_id, (simxInt)handle, -1, position, simx_opmode_oneshot);
    ret = simxSetObjectOrientation(m_vrep.client_id, (simxInt)handle, -1, orientation, simx_opmode_oneshot);
    if (ret!=simx_return_ok || res!= simx_return_ok) return false;
    else return true;
}

bool vrep_setpose(int link_id,Vector3d &pos,Vector3d & ori,int refhandle)
{
    bool ret = false;
    vector<double> orid = {ori[0],ori[1],ori[2]};
    vector<double> posi = {pos[0],pos[1],pos[2]};
    if(link_id>=1 && link_id<=6)
    {
//        cout<< "dyn.linkid is " << link_id << " vrep.linkhandle is " << m_vrep.link_handle[link_id-1] << endl;
        ret = vrep_setpos(m_vrep.link_handle[link_id-1],posi,orid,refhandle);
    }
    else  ret = vrep_setpos(m_vrep.goal_handle,posi,orid,refhandle);
    if(ret != true) return false;
    return true;
}

bool vrep_setpose(int link_id,Vector3d &pos,Matrix3d & mat,int refhandle)
{
    bool ret = false;
    vector<double> posi = {pos[0],pos[1],pos[2]};
    Vector3d rpy = mat.eulerAngles(0,1,2);
    vector<double> ori = {rpy[0],rpy[1],rpy[2]};
    if(link_id>=1 && link_id<=6)
    {
        ret = vrep_setpos(m_vrep.link_handle[link_id-1],posi,ori,refhandle);
    }
    else  ret = vrep_setpos(m_vrep.goal_handle,posi,ori,refhandle);
    if(ret != true) return false;
    return true;
}

void vrep_setrbdllinks()
{
    Vector3d pos;
    Vector3d ori;

    m_dyn.updatalinks();

    for(size_t n = 0; n < 6; n++)
    {
        //make sure linkid is 1~6 to set links in vrep.
        vrep_setpose(m_dyn.linkids[n],m_dyn.linkpos[n],m_dyn.linkrpy[n]);
        extApi_sleepMs(10);
    }
}

void vrep_setjoints(rbdlmath::VectorNd & jointrad)
{
    for(int i = 0;i<6;i++)
    {
        simxSetJointPosition(m_vrep.client_id, m_vrep.joint_handle[i], (simxFloat)jointrad[i], simx_opmode_oneshot);
    }
}

rbdlmath::VectorNd  validik(rbdlmath::Vector3d & endposv,rbdlmath::Matrix3d & endorim, int sig)
{


    double init[6] = {0,M_PI,M_PI,0,0,0};
    rbdlmath::VectorNd ikgoal(6);
    rbdlmath::VectorNd rbdlgoal(6);
    for(int i =0;i<6;i++) ikgoal[i] = init[i];

    int maxiteration = 10000;
    double threshold = 0.001;
    bool ikfound = false;
    bool collisionresult = true;

    //check rbdl solution first:
//    cout << " -------------------    finding possible ik solution         ----------------" <<endl;
    int rest = m_dyn.inverseKinematics(endposv,endorim,rbdlgoal,1);
    if(rest ==true)
    {
        m_dyn.updatalinks();
        m_collision.updatetransforms();
        collisionresult = collide();
    }

    if (collisionresult==false)
    {
//        cout << "rbdl original solution VALID : " << endl << radtoang(rbdlgoal) << endl;
        return rbdlgoal;
    }
    else
    {
        cout << endl<< endl<< endl<< "rbdl solution invalid, searching for possible solutions" << endl;
        bounds limit;
        std::vector<double> lowlimit = limit.low;
        std::vector<double> range = limit.range;
        cout << "lowlimit is : " << lowlimit[0] <<endl<< lowlimit[1] <<endl<<lowlimit[2] <<endl;
        cout << "range is : " << range[0] <<endl<< range[1] <<endl<<range[2] <<endl;
        rbdlmath::VectorNd jointdeg(6);
        rbdlmath::Vector3d tippos;
        rbdlmath::Vector3d tipori;
        rbdlmath::Vector3d endoriv = endorim.eulerAngles(0,1,2);

        for(int iterationcnt = 0; iterationcnt<maxiteration;iterationcnt++)
        {
            double tipdist = 0;
            double error[3] = {0,0,0};
            for(size_t j = 0;j<6;j++)
            {
                jointdeg[j] = lowlimit[j] + range[j]*(double)rand()/RAND_MAX;
    //            cout << " random joint angle degree is : " << jointdeg[j] << endl;
            }
            ikgoal = angtorad(jointdeg);
            m_dyn.setJointPositions(ikgoal);
            m_dyn.forwardKinematics(ikgoal,tippos, tipori);


            switch (sig)
            {
                case 1:
                {
                    tipdist = 0;
                    for(size_t a = 0;a<3;a++)
                    {
                        error[a] = endposv[a] - tippos[a];
                        tipdist += error[a]*error[a];
                    }
        //            cout << "distance is : " << tipdist <<endl;
                    if(tipdist<=threshold)
                        ikfound = true;
                    break;
                }
                case 2:
                {
                    tipdist = 0;
                    for(size_t a = 0;a<3;a++)
                    {
                        error[a] = endoriv[a] - tipori[a];
                        tipdist += error[a]*error[a];
                    }

                    if(tipdist<=threshold)
                        ikfound = true;
                    break;
                }
                case 3:
                {
                    tipdist = 0;
                    for(size_t a = 0;a<3;a++)
                    {
                        error[a] = endposv[a] - tippos[a];
                        tipdist += error[a]*error[a];
                    }
                    for(size_t a = 0;a<3;a++)
                    {
                        error[a] = endoriv[a] - tipori[a];
                        tipdist += error[a]*error[a];
                    }
                    if(tipdist<=threshold*2)
                        ikfound = true;
                    break;
                default: break;
                }
            }
            // using random ikgoal's forward kinematics as ik input, try rbdl again
            m_dyn.setJointPositions(ikgoal);
            m_dyn.inverseKinematics(endposv,endorim,rbdlgoal,1);
            m_dyn.setJointPositions(rbdlgoal);
            m_dyn.updatalinks();
            m_collision.updatetransforms();
            collisionresult = collide();
            if (collisionresult==false)
            {
                cout <<endl << endl <<endl <<  "random configure - rbdl solution VALID : searched for " << iterationcnt << " times." << endl << rbdlgoal << endl;
                return rbdlgoal;
            }
            else cout << "rbdl solution invalid" <<endl;
            if (ikfound ==true )
            {
                m_dyn.updatalinks();
                m_collision.updatetransforms();
                collisionresult = collide();
                cout << "found pure ik solution.............check collision?   " << endl;
                if (collisionresult == false)
                {
                    cout << endl<< endl<< endl<< endl<< "ik  solution VALID :" <<endl << ikgoal <<endl << " in degree :" <<radtoang(ikgoal)<< endl<<endl<<endl;
                    cout << "distance is : " << tipdist << endl;
                    cout << "iterated for " << iterationcnt << " times" << endl;
                    return ikgoal;
                }
                else
                {
                    cout << "ik  solution INVALID...." << endl<<endl<<endl;
                    ikfound = false;
                }
            }
        }
        cout << endl<< endl<< endl<< endl<<" no valid ik solution after iteration for " << maxiteration << " times" << endl;
        return ikgoal;
    }
}

bool obstaclemoved()
{

//    m_collision.obstacleobj->setTransform(m_collision.obstacletransforms.linear(),m_collision.obstacletransforms.translation());
//    m_collision.obstaclemanager->update(m_collision.obstacleobj);
//    int obstacle_num = m_collision.obstaclemanager->size();
//    for(int i =0;i<obstacle_num;i++)
//    {

//    }
}

double myPlanner::distanceFunction(const Motion *a, const Motion *b) const
{
//    cout << "calculate distance between sampled points" << endl;
    VectorNd aj(6);
    VectorNd bj(6);
    Vector3d endposa;
    Vector3d endposb;
    Vector3d endoria;
    Vector3d endorib;
    Vector3d error;
    double dis;
    //sample joint space points:
    for (int i = 0; i < 6; i++)
    {
        aj[i] = (float)(a->state->as<OmplStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(i)->values[0])*M_PI/180;
        bj[i] = (float)(b->state->as<OmplStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(i)->values[0])*M_PI/180;
    }
    m_dyn.setJointPositions(aj);
    m_dyn.getEndPose(endposa,endoria);
    m_dyn.setJointPositions(bj);
    m_dyn.getEndPose(endposb,endorib);
    for(int i = 0;i<3;i++)
    {
        error[i] = endposa[i] - endposb[i];
    }
    dis = sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
    return dis;
}


bool configendompl(og::SimpleSetup& setup,ob::SpaceInformationPtr& si,rbdlmath::Vector3d & startpos,rbdlmath::Vector3d &startori,rbdlmath::Vector3d &goalpos,rbdlmath::Vector3d &goalori)
{

    ob::ScopedState<endStateSpace> start(si);
    ob::ScopedState<ob::SE3StateSpace> goal(si);
    for(int i = 0;i<3;i++ )
    {
        start->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = startpos[i];
        goal->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = goalpos[i];
    }
    start->as<ob::SO3StateSpace::StateType>(3)->setIdentity();
    goal->as<ob::SO3StateSpace::StateType>(3)->setIdentity();
    setup.setStartAndGoalStates(start,goal);
    cout << " till now is ok >>>>>>>>>>>>>>>>>>>>>" << endl;

    //set state valid chacker and parameters

    si->setStateValidityChecker(std::make_shared<endStateValidityCheckerClass>(si));
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.05);


    cout << " ******************     setup information      *******************" << endl;
    setup.print(cout);

    cout << "-----------------  planner  ---------------" << endl;
    //using rrtconnnect algorithm

    auto planner(std::make_shared<og::TRRT>(si));
//    planner->setProblemDefinition(pdef);
//    planner->setup();

//    planner->printProperties(std::cout);
    setup.setPlanner(planner);

    cout << " ******************     setup information      *******************" << endl;
    setup.print(cout);


    return true;
}

void CalcDynamics(VectorNd q,VectorNd& tau)
{
    VectorNd qd      = VectorNd::Zero (m_dyn.dofs);
    VectorNd qdd     = VectorNd::Zero (m_dyn.dofs);
    static VectorNd q_old       = VectorNd::Zero (m_dyn.dofs);
    static VectorNd qd_old      = VectorNd::Zero (m_dyn.dofs);
    qd = (q - q_old)/T_SIM;
    qdd = (qd - qd_old)/T_SIM;
    q_old = q;
    qd_old = qd;
    m_dyn.inverseDynamics(q,qd,qdd,tau);
}

void rml_traj(std::vector<rbdlmath::VectorNd> & jointall)
{
    int num = jointall.size();
    m_traj.init(m_dyn.dofs,T_SIM);
    rbdlmath::VectorNd p(m_dyn.dofs),v(m_dyn.dofs),a(m_dyn.dofs);
    rbdlmath::VectorNd tau(m_dyn.dofs);
    vector<rbdlmath::VectorNd> traj_p;
    vector<rbdlmath::VectorNd> traj_v;
    vector<rbdlmath::VectorNd> traj_a;



    // move point to point, cv = tv = 0
    // pre calculate full trajectory for use later
    for(int n=0;n<num-1;n++)
    {
        m_traj.rmlPos(jointall[n],jointall[n+1],1,2);
        while(!m_traj.rmlStep(p,v,a))
        {
            CalcDynamics(p,tau);
            traj_p.push_back(p);
            traj_v.push_back(v);
            traj_a.push_back(a);
        }
    }

    int trajectorylength = traj_p.size();
    for(int i = 0;i<trajectorylength;i++)
    {
//        cout << "trajectory points " << i << " is : " <<endl << traj_p[i]<<endl;
        vrep_setjoints(traj_p[i]);
        extApi_sleepMs(10);
    }
}
void cubic_traj(std::vector<rbdlmath::VectorNd> & jointall)
{
    int num = jointall.size();
    vector<rbdlmath::VectorNd>  traj_p;
    vector<rbdlmath::VectorNd>  traj_v;
    vector<rbdlmath::VectorNd>  traj_a;
    vector<vector<double>>  t_p(6);
    vector<vector<double>>  t_v(6);
    vector<vector<double>>  t_a(6);
    int totallen =0;
    int timelength =0;
    for(int p = 0; p<num-1; p++)
    {


        for(int jp=0;jp<6;jp++)
        {
            double cp = jointall[p][jp];
            double tp = jointall[p+1][jp];
            cout << "current point : " << jp << ". joint " << p << ". pos is : " << cp << endl;
            double cv = 0;
            double tv = 0;
            double ct = 0;
            double tt = 5;
            double dt = 0.1;
            timelength = (tt-ct)/dt;
            m_cubic.calculatecubic(timelength,cp,ct,cv,tp,tt,tv,t_p[jp],t_v[jp],t_v[jp]);
            cout << "path of joint " << jp << " . sum length is :" << t_p[jp].size() << endl;
        }
        cout << "part : " << p+1 << " , lenth is : " << timelength << endl;

        totallen = totallen + timelength;
    }

    rbdlmath::VectorNd pt(6);
    for(int i=0;i<totallen;i++)
    {
        for(int j =0;j<6;j++)
        {
            pt[j] = t_p[j][i];
        }
        traj_p.push_back(pt);

    }
    cout << " total trajectory points : " <<  totallen << endl;
    for(int vp = 0; vp<totallen; vp++)
    {
        cout << "path point in joint space is : " << traj_p[vp] << endl;
        vrep_setjoints(traj_p[vp]);
        extApi_sleepMs(10);
    }
}
void quintic_traj(std::vector<rbdlmath::VectorNd> & jointall)
{
    int num = jointall.size();
    vector<rbdlmath::VectorNd>  traj_p;
    vector<rbdlmath::VectorNd>  traj_v;
    vector<rbdlmath::VectorNd>  traj_a;
    vector<vector<double>>  t_p(6);
    vector<vector<double>>  t_v(6);
    vector<vector<double>>  t_a(6);
    int totallen =0;

    cout << "quintic polynomial parts : " << num-1 << endl;

    vector<vector<double>> cp={{},{},{},{},{},{}};
    vector<vector<double>> cv(6);
    vector<vector<double>> ca(6);
    vector<vector<double>> tp={{},{},{},{},{},{}};
    vector<vector<double>> tv(6);
    vector<vector<double>> ta(6);
    vector<vector<double>> tt(6);
    vector<vector<double>> ct(6);
    double dt = 0.1;
    vector<int> T(6);
    for(int p = 0; p<num-1; p++)
    {
        cout << "calculate part : " << p << endl;
        cout << "current joints: " << endl << jointall[p] <<endl;
        for(int jp=0;jp<6;jp++)
        {
            cout << "parameters for joint : " << jp << endl;
            cp[jp].push_back(jointall[p][jp]);
            tp[jp].push_back(jointall[p+1][jp]);
            cout << cp[jp][p] << endl;
            cout << tp[jp][p] << endl;
            ca[jp].push_back(0);
            ta[jp].push_back(0);
            cout << ca[jp][p] << endl;
            cout << ta[jp][p] << endl;
            ct[jp].push_back(5*p);
            tt[jp].push_back(5*(p+1));
            cout << ct[jp][p] << endl;
            cout << tt[jp][p] << endl;
        }
        T[p] = (int)((tt[0][p]-ct[0][p])/dt);
        cout << "part : " << p << " time is : " << T[p] << endl;
    }
    cout << "tp length is : " << tp[0].size() << endl;
    cout << "ta length is : " << ta[0].size() << endl;
    cout << "tt length is : " << tt[0].size() << endl;

    for(int q = 0;q<num-2;q++)
    {
        for(int jp =0;jp<6;jp++)
        {
            double dot1 = (tp[jp][q]-cp[jp][q])/(tt[jp][q] - ct[jp][q]);
            double dot2 = (tp[jp][q+1]-cp[jp][q+1])/(tt[jp][q+1] - ct[jp][q+1]);

            if((dot1<0 && dot2 < 0) || (dot1>0 && dot2> 0))
            {
                if(q==0)cv[jp].push_back(0);
                if(q>0) cv[jp].push_back(tv[jp][q-1]);
                tv[jp].push_back((dot1+dot2)/2);
                if(q==num-3)
                {
                    cv[jp].push_back(tv[jp][num-3]);
                    tv[jp].push_back(0);
                }
            }
            else if(dot1*dot2<=0)
            {
                if(q==0)cv[jp].push_back(0);
                if(q>0) cv[jp].push_back(tv[jp][q-1]);
                tv[jp].push_back(0);
                if(q==num-3)
                {
                    cv[jp].push_back(tv[jp][num-3]);
                    tv[jp].push_back(0);
                }
            }

        }
    }
    cout << "cv length is : " << cv[0].size() << endl;
    cout << "tv length is : " << tv[0].size() << endl;
    for(int p = 0; p<num-1; p++)
    {


        for(int jp=0;jp<6;jp++)
        {
            m_cubic.calculatequintic(T[p],cp[jp][p],ct[jp][p],cv[jp][p],ca[jp][p],tp[jp][p],tt[jp][p],tv[jp][p],ta[jp][p],t_p[jp],t_v[jp],t_v[jp]);
            cout << "path of joint " << jp << " . sum length is :" << t_p[jp].size() << endl;
        }
        cout << "part : " << p+1 << " , lenth is : " << T[p] << endl;

        totallen = totallen + T[p];

    }

    rbdlmath::VectorNd pt(6);
    for(int i=0;i<totallen;i++)
    {
        for(int j =0;j<6;j++)
        {
            pt[j] = t_p[j][i];
        }
        traj_p.push_back(pt);

    }
    cout << " total trajectory points : " <<  totallen << endl;
    for(int vp = 0; vp<totallen; vp++)
    {
        cout << "path point in joint space is : " << traj_p[vp] << endl;
        vrep_setjoints(traj_p[vp]);
        extApi_sleepMs(10);
    }
}

bool checkdistance(rbdlmath::VectorNd & config1, rbdlmath::VectorNd & config2)
{
    double distance = 0;
    for(int i=0;i<6;i++)
    {
        distance = distance + (config1[i] - config2[i])*(config1[i] - config2[i]);
    }
    if (distance>=300) return false;
    else return true;
}

bool solveformaxtimes(ob::SpaceInformationPtr& si,og::SimpleSetup & setup, ob::PlannerStatus & solved,std::vector<rbdlmath::VectorNd> & jointall )
{
    int length = 10000;
    og::PathGeometric path(si);
    bool findpath = false;
     for(size_t i = 0;i<100;i++)
     {
        solved = setup.solve(10);
         if(setup.haveExactSolutionPath())
         {
             if(setup.getSolutionPath().length()<length)
             {
                 path = setup.getSolutionPath();
                 length = path.length();
                 cout << "update shortest path......  " << length << endl;
                 findpath = true;

             }
         }
     }

     if(findpath == false)
     {
         cout << " no solution founded" << endl;
         return false;
     }
     else
     {


         ob::ScopedState<OmplStateSpace> pathstate(setup.getStateSpace());
         int i = path.getStateCount();
         int len = path.length();
         /* interpolated in c space */
//         path.interpolate(6);

         i = path.getStateCount();
         len = path.length();
         cout << " interpolated path point number is : " << i << endl;
         cout << " interpolated path length is : " << len << endl;
         for(int c=0;c<i;c++)
         {
 //            cout<< "*******************  path point : " << c <<"  ********************"<<endl;
             pathstate = path.getState(c);
 //            setup.getStateSpace()->as<OmplStateSpace>()->ShowState(pathstate);
             rbdlmath::VectorNd jointd(6);
             rbdlmath::VectorNd jointabgd(6);
             for (int i = 0; i < 6; i++)
             {
                 jointabgd[i] = (double)pathstate->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
             }
             jointd = angtorad(jointabgd);
             jointall.push_back(jointd);

         }

//         quintic_traj(jointall);


         return true;

     }
}
#endif // MAIN_H
