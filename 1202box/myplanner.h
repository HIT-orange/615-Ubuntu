#ifndef MYPLANNER_H
#define MYPLANNER_H
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
//#include "dynamics.h"
//#include "OmplSpace.h"
namespace ob = ompl::base;
namespace og = ompl::geometric;

class myPlanner : public ob::Planner
{
public:
    myPlanner( ob::SpaceInformationPtr &si, std::string name, bool addIntermediateStates=false): ob::Planner(si, "plannernameunset")
    {
        specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
        specs_.directed = true;
        specs_.approximateSolutions = false;
        specs_.multithreaded = true;
        specs_.optimizingPaths = true;

        myPlanner::declareParam<double>("range", this, &myPlanner::setRange, &myPlanner::getRange, "0.:1.:10000.");
        myPlanner::declareParam<bool>("intermediate_states", this, &myPlanner::setIntermediateStates,
                                    &myPlanner::getIntermediateStates, "0,1");

        connectionPoint_ = std::make_pair<ob::State *, ob::State *>(nullptr, nullptr);
        distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
        addIntermediateStates_ = addIntermediateStates;
        myPlannerPtr_ = ob::PlannerPtr(this);
        setName(name);
    }

    ob::PlannerPtr getPlannerPtr()
    {
        return myPlannerPtr_;
    }

    ~myPlanner() override;

    void getPlannerData(ob::PlannerData &data) const override;

    ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

    void clear() override;

    bool getIntermediateStates() const
    {
        return addIntermediateStates_;
    }

    void setIntermediateStates(bool addIntermediateStates)
    {
        addIntermediateStates_ = addIntermediateStates;
    }

    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    double getRange() const
    {
        return maxDistance_;
    }

    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        tStart_ = std::make_shared<NN<Motion *>>();
        tGoal_ = std::make_shared<NN<Motion *>>();
        setup();
    }

    void setup() override;

protected:
    class Motion
    {
    public:
        Motion() = default;

        Motion(const ob::SpaceInformationPtr &si) : state(si->allocState())
        {
        }

        ~Motion() = default;

        const ob::State *root{nullptr};
        ob::State *state{nullptr};
        Motion *parent{nullptr};
    };

    using TreeData = std::shared_ptr<ompl::NearestNeighbors<Motion *>>;

    struct TreeGrowingInfo
    {
        ob::State *xstate;
        Motion *xmotion;
        bool start;
    };

    enum GrowState
    {
        TRAPPED,
        ADVANCED,
        REACHED
    };

    void freeMemory();

//    double distanceFunction(const Motion *a, const Motion *b) const
//    {
//        return si_->distance(a->state, b->state);
//    }
    double distanceFunction(const Motion *a, const Motion *b) const;


    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

    ob::StateSamplerPtr sampler_;

    TreeData tStart_;

    TreeData tGoal_;

    double maxDistance_{0.};

    bool addIntermediateStates_;

    ompl::RNG rng_;

    std::pair<ob::State *, ob::State *> connectionPoint_;

    double distanceBetweenTrees_;

    ob::PlannerPtr myPlannerPtr_;
};



#endif // MYPLANNER_H
