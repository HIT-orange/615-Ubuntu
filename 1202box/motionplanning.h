#ifndef MOTIONPLANNING_H
#define MOTIONPLANNING_H
//#include "kinematics.h"
//#include "globalDef.h"

//reflexx II releated
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>

//using namespace Eigen;

class motionplanning
{
public:
    motionplanning();
    void rmlPos(Eigen::VectorXd cp,Eigen::VectorXd tp,float maxVel = -1, float maxAccel = -1);
    bool rmlStep(Eigen::VectorXd& jp,Eigen::VectorXd& jv,Eigen::VectorXd& ja);
    void rmlRemove();
    //void rmlMoveToJointPositions(VectorXd cp,VectorXd tp,float t, float maxVel=-1, float maxAccel=-1);
    void init(int dofs, float T);
    //void rmlMoveToPosition(C7Vector transfrom);
private:
    // Variable declarations and definitions
    ReflexxesAPI*                 m_RML;
    RMLPositionInputParameters*   m_IP;
    RMLPositionOutputParameters*  m_OP;
    RMLPositionFlags              m_Flags;

    float   m_JointMaxVel;
    float   m_JointMaxAccel;
    float   m_EndMaxVel;
    float   m_EndMaxAccel;
    float   m_EndMaxJerk;

    int m_Dofs;
    float m_T;
};

#endif // MOTIONPLANNING_H
