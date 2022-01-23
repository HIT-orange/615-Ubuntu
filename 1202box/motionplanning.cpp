#include "motionplanning.h"
#include <unistd.h>

//#define CYCLE_TIME_IN_SECONDS       0.01f

motionplanning::motionplanning()
{
    m_RML = NULL;
    m_IP = NULL;
    m_OP = NULL;

    m_JointMaxVel = 0.5f;
    m_JointMaxAccel = 0.1f;
    m_EndMaxVel = 1.0f;
    m_EndMaxAccel = 1.0f;
}

void motionplanning::init(int dofs, float T)
{
    m_Dofs = dofs;
    m_T = T;
}

bool motionplanning::rmlStep(Eigen::VectorXd& jp,Eigen::VectorXd& jv,Eigen::VectorXd& ja)
{
    int                         ResultValue = 0;

    // Calling the Reflexxes OTG algorithm
    ResultValue = m_RML->RMLPosition(*m_IP
        , m_OP
        , m_Flags);

    if (ResultValue < 0)
    {
        printf("An error occurred (%d).\n", ResultValue);
    }

    *m_IP->CurrentPositionVector = *m_OP->NewPositionVector;
    *m_IP->CurrentVelocityVector = *m_OP->NewVelocityVector;
    *m_IP->CurrentAccelerationVector = *m_OP->NewAccelerationVector;

    for (int i = 0; i < m_Dofs; i++)
    {
        jp[i] = m_OP->NewPositionVector->VecData[i];
        jv[i] = m_OP->NewVelocityVector->VecData[i];
        ja[i] = m_OP->NewAccelerationVector->VecData[i];
    }

    return (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}

void motionplanning::rmlPos(Eigen::VectorXd cp,Eigen::VectorXd tp,float maxVel, float maxAccel)
{
    // move point to point, with cv = 0 and tv = 0
    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    m_RML = new ReflexxesAPI(m_Dofs, m_T);

    m_IP = new RMLPositionInputParameters(m_Dofs);

    m_OP = new RMLPositionOutputParameters(m_Dofs);

    for (int i = 0; i < m_Dofs; i++)
    {
        m_IP->CurrentPositionVector->VecData[i] = cp[i];
        m_IP->CurrentVelocityVector->VecData[i] = 0;
        m_IP->CurrentAccelerationVector->VecData[i] = 0;
        m_IP->MaxVelocityVector->VecData[i] = maxVel;
        m_IP->MaxAccelerationVector->VecData[i] = maxAccel;
        //m_IP->MaxJerkVector->VecData[i] = 100;
        m_IP->TargetPositionVector->VecData[i] = tp[i];
        m_IP->TargetVelocityVector->VecData[i] = 0;
        m_IP->SelectionVector->VecData[i] = true;
    }
}

void motionplanning::rmlRemove()
{
    delete  m_RML;
    delete  m_IP;
    delete  m_OP;
}
//*********************************************************************************************
//
