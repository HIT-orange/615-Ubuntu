#include "main.h"


using namespace fcl;

extern dynamics m_dyn;
extern cs::collision m_collision;
extern vp::VrepShow m_vrep;
extern polynomial m_cubic;
extern motionplanning m_traj;

bool jointspaceompltest()
{
    /*initialize rdbl model,ompl planner, fcl collision model, and vrep communication*/
    initall();

    /*ompl initialize :*/
    bounds limits;
    ob::StateSpacePtr jointspace(new OmplStateSpace(&limits));
    og::SimpleSetup setup(jointspace);
    ob::SpaceInformationPtr si(setup.getSpaceInformation());
    ob::PlannerStatus solved;
    std::vector<rbdlmath::VectorNd> jointall;


    /*set start position-rbdl,vrep,fcl*/
    float startjointpos[6] = {0,M_PI,M_PI,0,0,0};
    rbdlmath::VectorNd startposrad(6);
    floattovector(startjointpos,startposrad);
    setstart(startposrad);//joint position in rad

    /*set goal position-vrep goal dummy*/
    rbdlmath::Vector3d endposv = {-0.45,0.25,0.2};
    rbdlmath::Vector3d endrpy = {0,M_PI,0};
    rbdlmath::Matrix3d fakeendori = m_dyn.rpyToMatrix(endrpy);
    rbdlmath::VectorNd goalposrad(6);
    goalposrad = validik(endposv,fakeendori,1);
    setgoal(endposv,goalposrad);//end position (and ori) and joint position in rad

    /*configure ompl start and goal */
    configkinovaompl(setup,si,startposrad,goalposrad);//input joint position in rad, inner transform



    /*using setup.solved with rrtconnect algorithm to find a better solution*/
    bool res = solveformaxtimes(si,setup,solved, jointall);


    /*set goal position-vrep goal dummy*/
    ob::StateSpacePtr jointspace2(new OmplStateSpace(&limits));
    og::SimpleSetup setup2(jointspace2);
    ob::SpaceInformationPtr si2(setup2.getSpaceInformation());
    ob::PlannerStatus solved2;

    rbdlmath::Vector3d endposv2 = {0.45,0.25,0.2};
    rbdlmath::Vector3d endrpy2 = {0,0,0};
    rbdlmath::Matrix3d fakeendori2 = m_dyn.rpyToMatrix(endrpy);
    rbdlmath::VectorNd goalposrad2(6);
    goalposrad2 = validik(endposv2,fakeendori2,1);
    setgoal(endposv2,goalposrad2);//end position (and ori) and joint position in rad

    setup.clear();

    /*configure ompl start and goal */
    configkinovaompl(setup2,si2,goalposrad,goalposrad2);//input joint position in rad, inner transform

    /*using setup.solved with rrtconnect algorithm to find a better solution*/
    std::vector<rbdlmath::VectorNd> jointall2;
    res = solveformaxtimes(si2,setup2,solved2, jointall2);

//    showpath(setup,1);
//    m_vrep.vrep_stop();
//    extApi_sleepMs(1000);
//    m_vrep.vrep_start();
//    showpath(setup,3);

//    rml_traj(jointall);
//    rml_traj(jointall2);
    quintic_traj(jointall);
    quintic_traj(jointall2);

    return 0;
}


void endspaceompltest()
{
    initall();
    /*set moveable obstacle transforms*/
    fcl::Vector3d obstacleposition = {0,0.54,0.2};
    fcl::Vector3d obstacleori = {0,0,M_PI/2};
    fcl::Matrix3d obstacleorientation = m_dyn.rpyToMatrix(obstacleori);
    m_collision.obstacletransforms[0].linear() = obstacleorientation;
    m_collision.obstacletransforms[0].translation() = obstacleposition;
    m_collision.updateobstacle();

    // construct the state space we are planning in
    ob::StateSpacePtr endspace(new endStateSpace());
    og::SimpleSetup setup(endspace);
    auto si(setup.getSpaceInformation());
    ob::PlannerStatus solved;

    /*set start position-rbdl,vrep,fcl*/
    float startjointpos[6] = {0,M_PI,M_PI,0,0,0};
    rbdlmath::VectorNd startposrad(6);
    floattovector(startjointpos,startposrad);
//    endsetstart(startposrad);
    rbdlmath::Vector3d startposv = rbdlmath::Vector3d::Zero();
    rbdlmath::Matrix3d startorim = rbdlmath::Matrix3d::Identity();
    m_dyn.fkgetlinkpose(startposrad,m_dyn.linkids[5],startposv,startorim);
    rbdlmath::Vector3d startoriv = startorim.eulerAngles(0,1,2);

    //set vrep goal dummy
    rbdlmath::Vector3d goalposv = {-0.45,0.25,0.2};
    rbdlmath::Vector3d endrpy = rbdlmath::Vector3d::Identity();
    vp::pos3d pose;
    pose.posx=goalposv[0];
    pose.posy=goalposv[1];
    pose.posz=goalposv[2];
    float fakerpy[3]={0,0,0};
    m_vrep.vrep_setpose3d(m_vrep.goal_handle,-1,pose,fakerpy);

    rbdlmath::Matrix3d goalorim = m_dyn.rpyToMatrix(endrpy);
    rbdlmath::VectorNd endgoaljointrad(6);
    endgoaljointrad = validik(goalposv,goalorim,1);
    m_dyn.setJointPositions(startposrad);
    m_dyn.fkgetlinkpose(endgoaljointrad,m_dyn.linkids[5],goalposv,goalorim);
    rbdlmath::Vector3d goaloriv = goalorim.eulerAngles(0,1,2);

//    vrep_setjoints(goalposrad);
    m_dyn.setJointPositions(endgoaljointrad);
    m_dyn.updatalinks();
    m_collision.updatetransforms();
    m_collision.fclshow();

    bool result = collide();
    cout << " ________________________________    colliding result ? : " << result << endl;
    configendompl(setup,si,startposv,startoriv,goalposv,goaloriv);

//    cout << " solving ... " << endl;
////    extApi_sleepMs(2000);
//    solved = setup.solve(5.0);
//    if(solved )
//    {
//        cout << "solution founded " << endl;
//        showendpath(setup,3);
//    }
//    else cout<< " no solution " << endl;

    /*using setup.golved with rrtconnect algorithm to find a better solution*/
   int length = 10000;
   og::PathGeometric path(si);
   bool findpath = false;
    for(size_t i = 0;i<200;i++)
    {
       solved = setup.solve(1);
        if(setup.haveExactSolutionPath())
        {
            if(setup.getSolutionPath().length()<length)
            {
                path = setup.getSolutionPath();
                length = setup.getSolutionPath().length();
                cout << "update shortest path......  " << length << endl;
                findpath = true;

            }
        }
    }

    if(findpath == false) cout << " no solution founded" << endl;
    else
    {
        rbdlmath::VectorNd lastjoints=startposrad;
        std::vector<rbdlmath::VectorNd> jointall;
        jointall.push_back(startposrad);
        ob::ScopedState<endStateSpace> s(setup.getStateSpace());
        int i = path.getStateCount();
        int len = path.length();
        /* interpolated in c space */
        path.interpolate(100);
        i = path.getStateCount();
        len = path.length();
        cout << " interpolated path point number is : " << i << endl;
        cout << " interpolated path length is : " << len << endl;
        for(int c=0;c<i;c++)
        {
//            cout<< "*******************  path point : " << c <<"  ********************"<<endl;
            s = path.getState(c);
            rbdlmath::Vector3d endposv={0,0,0};
            rbdlmath::Matrix3d fakeorim = rbdlmath::Matrix3d::Identity();
            for(int i =0;i<3;i++)
            {
                endposv[i] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
            }

            rbdlmath::VectorNd joints = validik(endposv,fakeorim,1);

//            bool res = checkdistance(joints,lastjoints);
//            if (res == true)
//            {
//                m_dyn.setJointPositions(joints);
//                m_dyn.updatalinks();
//                jointall.push_back(joints);
//                lastjoints = joints;
//            }
//            else continue;


//            setup.getStateSpace()->as<endStateSpace>()->ShowState(pathstate);
        }

        /*using rml to follow joints*/

        jointall.push_back(endgoaljointrad);

        rml_traj(jointall);
//        cubic_traj(jointall);
//        quintic_traj(jointall);

    }


}

int main()
{
    jointspaceompltest();


//    endspaceompltest();







//    system("pause");
    return 0;
}



