#include "dynamics.h"
//====================================================================
// Boost stuff
//====================================================================

//typedef std::vector< double > state_type;

//typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
//typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;


//class rbdlToBoost {

//public:
//    rbdlToBoost(Model* model) : model(model) {
//        q = VectorNd::Zero(model->dof_count);
//        qd = VectorNd::Zero(model->dof_count);
//        qdd = VectorNd::Zero(model->dof_count);
//        tau = VectorNd::Zero(model->dof_count);

//    }

//    //3c. Boost uses this 'operator()' function to evaluate the state
//    //    derivative of the pendulum.
//    void operator() (const state_type &x,
//                     state_type &dxdt,
//                     const double t){

//        //3d. Here we split out q (generalized positions) and qd
//        //    (generalized velocities) from the x (state vector)
//        //q
//        int j = 0;
//        for(int i=0; i<model->dof_count; i++){
//            q[i] = (double)x[j];
//            j++;
//        }

//        //qd
//        for(int i=0; i<model->dof_count; i++){
//            qd[i] = (double)x[j];
//            j++;
//        }

//        //3e. Here we set the applied generalized forces to zero
//        for(int i=0; i<model->dof_count; i++){
//            tau[i] = 0;
//        }

//        //3f. RBDL's ForwardDynamics function is used to evaluate
//        //    qdd (generalized accelerations)
//        ForwardDynamics (*model,q,qd,tau,qdd);

//        //3g. Here qd, and qdd are used to populate dxdt
//        //(the state derivative)
//        j = 0;
//        for(int i = 0; i < model->dof_count; i++){
//            dxdt[j] = (double)qd[i];
//            j++;
//        }
//        for(int i = 0; i < model->dof_count; i++){
//            dxdt[j] = (double)qdd[i];
//            j++;
//        }


//    }

//private:
//    Model* model;
//    VectorNd q, qd, qdd, tau;
//};

//struct pushBackStateAndTime
//{
//    std::vector< state_type >& states;
//    std::vector< double >& times;

//    pushBackStateAndTime( std::vector< state_type > &states ,
//                          std::vector< double > &times )
//        : states( states ) , times( times ) { }

//    void operator()( const state_type &x , double t )
//    {
//        states.push_back( x );
//        times.push_back( t );
//    }
//};

//void f(const state_type &x, state_type &dxdt, const double t);
Matrix3d dynamics::rpyToMatrix(Vector3d rpy)
{
    Eigen::Matrix3d matrix;
    matrix = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
    //std::cout<<matrix.transpose()<<std::endl;
    return matrix;
}

void dynamics::inverseDynamics(    const VectorNd &Q,
                                   const VectorNd &QDot,
                                   const VectorNd &QDDot,
                                   VectorNd &Tau,
                                   std::vector<SpatialVector> *f_ext)
{
    UpdateKinematics(*model, q, qd,qdd);
    InverseDynamics(*model,Q,QDot,QDDot,Tau,f_ext);
}

void dynamics::forwardDynamics (
        const VectorNd &Q,
        const VectorNd &QDot,
        const VectorNd &Tau,
        VectorNd &QDDot,
        std::vector<SpatialVector> *f_ext )
{
    ForwardDynamics (*model,Q,QDot,Tau,QDDot);
}
bool dynamics::limitate(VectorNd& q, VectorNd &qres)
{

    for(size_t j=1;j<3;j++)
    {
//        std::cout << " ik init qres[ " <<j<< "] : "<<qres[j] << " in degree: " << qres[j]*180/M_PI << std::endl;
        while(qres[j]>=(M_PI*2))
        {
            qres[j] = qres[j] - M_PI*2;
//            std::cout << "+-M_PI*2" << std::endl;
        }
        while (qres[j]<=0)
        {
            qres[j] = qres[j] + M_PI*2;
//            std::cout << "+-M_PI*2" << std::endl;
        }              

//        std::cout << " ik result qres[ " <<j<< "] : "<<qres[j] << " in degree: " << qres[j]*180/M_PI << std::endl;
    }
    int j[4] = {0,3,4,5};
    for(int i:j)
    {
        while((qres[i])>2*M_PI)
        {
            qres[i] = qres[i]-M_PI*2;

        }
        while((qres[i])<-2*M_PI)
        {
            qres[i] = qres[i]+M_PI*2;
        }
//        while((qres[i]-q[i])>M_PI)
//        {
//            qres[i] = qres[i]-M_PI*2;

//        }
//        while((q[i]-qres[i])>M_PI)
//        {
//            qres[i] = qres[i]+M_PI*2;
//        }
//        std::cout << "limiting qres[ "<< i << " ] is : " << qres[i] << std::endl;
    }

    if(qres[1]>M_PI*310/180) return false;
    if(qres[1]<M_PI*50/180) return false;
    if(qres[2]>M_PI*341/180) return false;
    if(qres[2]<M_PI*19/180) return false;

    return true;
}
bool dynamics::inverseKinematics(Vector3d pos, Matrix3d ori,VectorNd& qres, int sig)
{
    //Vector3d local_point = Vector3d::Zero();
    InverseKinematicsConstraintSet cs;
    cs.ClearConstraints();
    switch (sig)
    {
    case(1): cs.AddPointConstraint(end,endpoint,pos);  break;//end pos constraint: translation
        case(2): cs.AddOrientationConstraint(end,ori);      break; //end ori constraint: rotation
        case(3): cs.AddFullConstraint(end,endpoint,pos,ori);break;
        default: break;
    }

    bool result=0;
//    while(result!=1)
//    {
        result = InverseKinematics (*model,q,cs,qres);
        if (result==0) return false;
//        printf("inverseKinematics result:%d\n",result);
//    }

    bool res = limitate(q,qres);
    if (res == true)
    {
        q = qres;
    //    for(size_t j=0;j<6;j++)
    //    {
    //        std::cout << " ik limited  q[ " <<j<< "] : degree: " << q[j]*180/M_PI << std::endl;
    //    }
        return true;
    }
    else return false;
}

bool dynamics::inverseKinematics(Vector3d pos, Vector3d rpy,VectorNd& qres, int sig)
{
    return inverseKinematics(pos,rpyToMatrix(rpy),qres, sig);
}

void dynamics::fkgetlinkpose(VectorNd qin, uint link_id, Vector3d& endpos,Matrix3d& endori)
{

    endpos =  CalcBodyToBaseCoordinates(*model, qin, link_id, linkref[link_id-1],true);
    endori =  CalcBodyWorldOrientation(*model, qin, link_id,true);
}

void dynamics::forwardKinematics(VectorNd qin, Vector3d& endpos,Matrix3d& endori)
{
    endpos =  CalcBodyToBaseCoordinates(*model,qin,end,endpoint);
    endori =  CalcBodyWorldOrientation(*model,qin,end);
}

void dynamics::forwardKinematics(VectorNd qin, Vector3d& endpos,Vector3d& endrpy)
{
    endpos =  CalcBodyToBaseCoordinates(*model,qin,end,endpoint);
    endrpy =  CalcBodyWorldOrientation(*model,qin,end).eulerAngles(0, 1, 2);
}

void dynamics::getEndPose(Vector3d& pos, Matrix3d& ori)
{
    pos =  CalcBodyToBaseCoordinates(*model,q,end,endpoint,true);
    ori =  CalcBodyWorldOrientation(*model,q,end);

    Vector3d euler = ori.eulerAngles(0, 1, 2);
    //output:
    std::cout << "endpos:" << std::endl << pos << std::endl;
    std::cout << "endori" << std::endl << euler << std::endl;

}

void dynamics::getEndPose(Vector3d& pos, Vector3d& rpy)
{
    getEndPosition(pos);
    getEndOrientation(rpy);
}

void dynamics::getEndPosition(Vector3d& pos)
{
    pos =  CalcBodyToBaseCoordinates(*model,q,end,endpoint);
}
void dynamics::getEndOrientation(Vector3d& rpy)
{
    rpy =  CalcBodyWorldOrientation(*model,q,end).eulerAngles(0, 1, 2);

}

void dynamics::setEndPose(const Vector3d pos, const Matrix3d ori)
{
    VectorNd qres;
    inverseKinematics(pos,ori,qres,1);
}

void dynamics::setEndPose(const Vector3d pos, const Vector3d rpy)
{
    setEndPose(pos,rpyToMatrix(rpy));
}

void dynamics::setJointPositions(const VectorNd jp)
{
    q = jp;
}

void dynamics::getJointPositions(VectorNd& jp)
{
    jp = q;
}

int dynamics::getDofs()
{
    return model->dof_count;
}
void dynamics::updatalinks()
{

    for(int i =0;i<6;i++)
    {
//        std::cout<<"link id: " << linkids[i] <<  ".  q["<<i<<"] : " << q[i] << std::endl;

        linkpos[i] = CalcBodyToBaseCoordinates(*model,q,linkids[i],linkref[i],true);
        linkposm[i] = rpyToMatrix(linkpos[i]);
//        std::cout << "pos : " << std::endl << linkpos[i] << std::endl;
//        std::cout << " ************************************************************************************" << std::endl;
//        Matrix3d midmatrix = CalcBodyWorldOrientation(*model, q, linkids[i+6], true);
//        std::cout << "joint matrix from forward kinematics : " << linkids[i]<< std::endl << midmatrix << std::endl;
//         midmatrix = CalcBodyWorldOrientation(*model, q, linkids[i], true);
//        std::cout << "link matrix from forward kinematics : "  << std::endl << midmatrix << std::endl;


//        linkquat[i] = Eigen::Quaterniond(linkrpyref[i]);

//        Matrix3d rpymat = midmatrix*linkrpyref[i];
//        linkrpy[i] = rpymat.eulerAngles(0,1,2);

//        std::cout << "final link ori : " << "R : "<< linkrpy[i].x()*180/M_PI  << "  P: " << linkrpy[i].y()*180/M_PI << "  Y: " << linkrpy[i].z()*180/M_PI << std::endl;
//        linkrpy[i][2] = - linkrpy[i][2];
//        std::cout << "final link ori : " << "R : "<< linkrpy[i].x()*180/M_PI  << "  P: " << linkrpy[i].y()*180/M_PI << "  Y: " << linkrpy[i].z()*180/M_PI << std::endl;

    }
    Matrix3d link21z = Eigen::AngleAxisd(q[0],Vector3d(0,0,1)).toRotationMatrix();
    Matrix3d link22z = Eigen::AngleAxisd(q[1],Vector3d(0,0,1)).toRotationMatrix();
    Matrix3d link2x = Eigen::AngleAxisd(-M_PI/2,Vector3d(1,0,0)).toRotationMatrix();
    Matrix3d linkx = Eigen::AngleAxisd(M_PI,Vector3d(1,0,0)).toRotationMatrix();
    linkrpym[1] = linkx*link21z*link2x*link22z*link2x;
    linkrpy[1] = linkrpym[1].eulerAngles(0,1,2);
    Matrix3d link3x = Eigen::AngleAxisd(M_PI/2,Vector3d(1,0,0)).toRotationMatrix();
    Matrix3d link3z = Eigen::AngleAxisd(q[2],Vector3d(0,0,1)).toRotationMatrix();
    linkrpym[2] = linkx*link21z*link2x*link22z*linkx*link3z*link3x;
    linkrpy[2] = linkrpym[2].eulerAngles(0,1,2);
}

bool dynamics::init()
{
    rbdl_check_api_version (RBDL_API_VERSION);

    model         = new Model();

    //3a. The URDF model is read in here, and turned into a series of
    //    vectors and matricies in model which RBDL uses to evaluate
    //    dynamics quantities
//    const char* filename = "../0909-3.urdf";
//    const char* filename = "../urdf-handwrite.urdf";
    const char* filename = "../urdf-link.urdf";

    if (!Addons::URDFReadFromFile (filename, model, false)) {
        std::cerr << "Error loading model " << filename << std::endl;
        abort();
    }
    else std::cout << "loading model successfully" << std::endl;
    model->gravity = Vector3d::Zero();
//    std::cout << "Degree of freedom overview:" << std::endl;
//    std::cout << Utils::GetModelDOFOverview(*model);

//    std::cout << "Model Hierarchy:" << std::endl;
//    std::cout << Utils::GetModelHierarchy(*model);
    dofs     = model->dof_count;
//    printf("DoF: %i\n",dofs);

    //set init joints angle
    q = VectorNd::Zero(dofs);
    qd = VectorNd::Zero(dofs);
    qdd = VectorNd::Zero(dofs);

//    q[1] = M_PI/2;
//    q[2] = M_PI;

    linkids[0] = model->GetBodyId("link_1");
    linkids[1] = model->GetBodyId("link_2");
    linkids[2] = model->GetBodyId("link_3");
    linkids[3] = model->GetBodyId("link_4");
    linkids[4] = model->GetBodyId("link_5");
    linkids[5] = model->GetBodyId("link_6");

//    linkids[0] = model->GetBodyId("link_1pos");
//    linkids[1] = model->GetBodyId("link_2pos");
//    linkids[2] = model->GetBodyId("link_3pos");
//    linkids[3] = model->GetBodyId("link_4pos");
//    linkids[4] = model->GetBodyId("link_5pos");
//    linkids[5] = model->GetBodyId("link_6pos");

//    for(int i =0;i<=11;i++)
//    {
//        std::cout << "lindid[ " << i << " ] is : " << linkids[i] << std::endl;
//    }

//    std::cout << "joint number is : " << model->q_size << std::endl;
//    for(int i = 0;i<model->q_size;i++)
//    {
//        std::cout << "joint [ " << i << "] is : " << q[i] << std::endl;
//    }

    updatalinks();

    end = model->GetBodyId("link_6");
//    endpoint = CalcBodyToBaseCoordinates(*model, q, end, {0,0,0}, true);
    endpoint = {0,0,-0.10};
    uint tip = model->GetBodyId("link_6");
    Vector3d tiptobase = CalcBodyToBaseCoordinates(*model, q, tip, {0,0,0}, true);
    std::cout<<"initial end point" << tiptobase<<std::endl;
    return true;

}

dynamics::dynamics()
{
    model = NULL;
}
