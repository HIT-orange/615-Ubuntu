#include "myplanner.h"



myPlanner::~myPlanner()
{
    freeMemory();
}

void myPlanner::setup()
{
    Planner::setup();
    ompl::tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void myPlanner::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
         tStart_->list(motions);
         for (auto &motion : motions)
         {
             if (motion->state != nullptr)
                 si_->freeState(motion->state);
             delete motion;
         }
    }

     if (tGoal_)
     {
         tGoal_->list(motions);
         for (auto &motion : motions)
         {
             if (motion->state != nullptr)
                 si_->freeState(motion->state);
             delete motion;
         }
     }
}

void myPlanner::clear()
{
     Planner::clear();
     sampler_.reset();
     freeMemory();
     if (tStart_)
         tStart_->clear();
     if (tGoal_)
         tGoal_->clear();
     connectionPoint_ = std::make_pair<ob::State *, ob::State *>(nullptr, nullptr);
     distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

myPlanner::GrowState myPlanner::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
     bool reach = true;

    /* find state to add */
    ob::State *dstate = rmotion->state;

//    double d = si_->distance(nmotion->state, rmotion->state);
    /*change distance function to calculate the Cartesian space distance*/
    double d = distanceFunction(nmotion,rmotion);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

         /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
         if (si_->equalStates(nmotion->state, tgi.xstate))
             return TRAPPED;


        dstate = tgi.xstate;
         reach = false;
    }

     bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                                si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (!validMotion)
         return TRAPPED;

     if (addIntermediateStates_)
     {
         const ob::State *astate = tgi.start ? nmotion->state : dstate;
         const ob::State *bstate = tgi.start ? dstate : nmotion->state;

         std::vector<ob::State *> states;
         const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);

         if (si_->getMotionStates(astate, bstate, states, count, true, true))
             si_->freeState(states[0]);

         for (std::size_t i = 1; i < states.size(); ++i)
         {
             auto *motion = new Motion;
             motion->state = states[i];
             motion->parent = nmotion;
             motion->root = nmotion->root;
             tree->add(motion);

             nmotion = motion;
         }

         tgi.xmotion = nmotion;
     }
     else
     {
         auto *motion = new Motion(si_);
         si_->copyState(motion->state, dstate);
         motion->parent = nmotion;
         motion->root = nmotion->root;
         tree->add(motion);

         tgi.xmotion = motion;
     }

    return reach ? REACHED : ADVANCED;
}

ob::PlannerStatus myPlanner::solve(const ob::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const ob::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
    (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    ob::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    while (!ptc)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const ob::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
            si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)
            gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                 OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<og::PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (!startTree)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
    tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<og::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
        path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return ob::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}

void myPlanner::getPlannerData(ob::PlannerData &data) const
{
     Planner::getPlannerData(data);

     std::vector<Motion *> motions;
     if (tStart_)
         tStart_->list(motions);

     for (auto &motion : motions)
     {
         if (motion->parent == nullptr)
             data.addStartVertex(ob::PlannerDataVertex(motion->state, 1));
         else
         {
             data.addEdge(ob::PlannerDataVertex(motion->parent->state, 1), ob::PlannerDataVertex(motion->state, 1));
         }
     }

     motions.clear();
     if (tGoal_)
         tGoal_->list(motions);

     for (auto &motion : motions)
     {
         if (motion->parent == nullptr)
             data.addGoalVertex(ob::PlannerDataVertex(motion->state, 2));
         else
         {
             // The edges in the goal tree are reversed to be consistent with start tree
             data.addEdge(ob::PlannerDataVertex(motion->state, 2), ob::PlannerDataVertex(motion->parent->state, 2));
         }
     }

     // Add the edge connecting the two trees
     data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

     // Add some info.
     data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}


