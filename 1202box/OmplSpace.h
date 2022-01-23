#ifndef OMPLSPACE_H
#define OMPLSPACE_H


#include <stdlib.h>
#include <iostream>
#include <stdio.h>

#include <config.h>
#include <StateSpace.h>
#include <PathSimplifier.h>
#include <RRTConnect.h>
#include <RealVectorStateSpace.h>
#include <StateSpaceTypes.h>
#include <StateSpaceTypes.h>
#include <ScopedState.h>
#include <StateSampler.h>
#include <SimpleSetup.h>
#include <RRTstar.h>
#include <BiTRRT.h>
#include <StateValidityChecker.h>
#include <State.h>
#include <SE3StateSpace.h>
#include <SO3StateSpace.h>
#include <TRRT.h>
#include <planners/rrt/TRRT.h>
#include <SpaceInformation.h>
#include <Planner.h>

#include <planners/rrt/InformedRRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;



struct bounds {
//    std::vector<double> low = { -360,50,19,-360,-360,-360 };
//    std::vector<double> high = { 360,310,341,360,360,360 };
    std::vector<double> low = { -360,0,0,-360,-360,-360 };
    std::vector<double> high = { 360,360,360,360,360,360 };
    std::vector<double> range = {high[0]-low[0],
                                 high[1]-low[1],
                                 high[2]-low[2],
                                 high[3]-low[3],
                                 high[4]-low[4],
                                 high[5]-low[5]};
};


class OmplStateSpace : public ob::CompoundStateSpace
{
public:
    OmplStateSpace(bounds* jointslimit) : ob::CompoundStateSpace(),jointslimit(jointslimit)
    {
        setName("JointCompoundStateSpace");
        char name[64];
        for (int i = 0; i < 6; i++)
        {
            sprintf(name, "joint%d", i + 1);
            ob::StateSpacePtr jp = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
            jp->setName(name);
            addSubspace(jp, 1);

            ob::RealVectorBounds bounds(1);
            bounds.setLow(jointslimit->low[i]);
            bounds.setHigh(jointslimit->high[i]);
            as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
        }
        setSubspaceWeight(0,8);
        setSubspaceWeight(1,6);
        setSubspaceWeight(2,4);
    }

    void ShowState(const ob::ScopedState<OmplStateSpace>& s);

protected:
    bounds* jointslimit;

};

class myStateValidityCheckerClass : public ob::StateValidityChecker
{
public:
     myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) :
       ob::StateValidityChecker(si)
     {

     }

     bool isValid(const ob::State *state) const;
//     bool isValid(const ob::State *state,double& dist) const;
};


class endStateSpace : public ob::CompoundStateSpace
{
public:
    endStateSpace() : ob::CompoundStateSpace()
    {
        setName("Kinova-End-StateSpace");

            ob::StateSpacePtr X = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
            X->setName("X");
            addSubspace(X, 1);
            ob::RealVectorBounds bounds(1);
            bounds.setLow(-1);
            bounds.setHigh(1);
            as<ob::RealVectorStateSpace>(0)->setBounds(bounds);

            ob::StateSpacePtr Y = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
            Y->setName("Y");
            addSubspace(Y, 1);
            bounds.setLow(-1);
            bounds.setHigh(1);
            as<ob::RealVectorStateSpace>(1)->setBounds(bounds);

            ob::StateSpacePtr Z = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
            Z->setName("Z");
            addSubspace(Z, 1);
            bounds.setLow(-1);
            bounds.setHigh(1.2);
            as<ob::RealVectorStateSpace>(2)->setBounds(bounds);

            ob::StateSpacePtr R = ob::StateSpacePtr(new ob::SO3StateSpace());
            R->setName("ROTATE QUAN");
            addSubspace(R, 1);
    }

    void ShowState(const ob::ScopedState<endStateSpace>& s);

};


class endStateValidityCheckerClass : public ob::StateValidityChecker
{
public:
     endStateValidityCheckerClass(const ob::SpaceInformationPtr &si) :
       ob::StateValidityChecker(si)
     {

     }

     bool isValid(const ob::State *state) const;

};



#endif // OMPLSPACE_H
