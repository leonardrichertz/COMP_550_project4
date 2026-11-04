///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#include "RG-RRT.h"

#include <limits>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/SelfConfig.h>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<unsigned int>("reach_steps", this, &RGRRT::setReachSteps, &RGRRT::getReachSteps,
                                        "1:1:50");
    Planner::declareParam<unsigned int>("num_controls", this, &RGRRT::setNumControls, &RGRRT::getNumControls,
                                        "2:1:51");
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();

    // RG-RRT: R(q) is built by sweeping a RealVectorControlSpace dimension between its bounds
    if (dynamic_cast<const RealVectorControlSpace *>(siC_->getControlSpace().get()) == nullptr)
        throw Exception(getName(), "requires a RealVectorControlSpace");
    if (numControls_ < 2)
        throw Exception(getName(), "num_controls must be at least 2");

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            // RG-RRT: the reachable states are owned by their motion
            for (auto *state : motion->reachable)
                si_->freeState(state);
            delete motion;
        }
    }
}

void ompl::control::RGRRT::computeReachableSet(Motion *motion) const
{
    const auto &bounds = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds();

    Control *control = siC_->allocControl();
    double *u = control->as<RealVectorControlSpace::ControlType>()->values;

    // Hold all other control dimensions at the middle of their range (0 for symmetric bounds)
    for (std::size_t d = 1; d < bounds.low.size(); ++d)
        u[d] = 0.5 * (bounds.low[d] + bounds.high[d]);

    // Sweep dimension 0 over numControls_ evenly spaced values, e.g. {-3, -2.4, ..., 2.4, 3} for torque 3
    const double step = (bounds.high[0] - bounds.low[0]) / (numControls_ - 1);
    for (unsigned int i = 0; i < numControls_; ++i)
    {
        u[0] = bounds.low[0] + i * step;

        base::State *result = si_->allocState();
        siC_->propagate(motion->state, control, reachSteps_, result);

        // An invalid state is certainly not reachable
        if (si_->isValid(result))
            motion->reachable.push_back(result);
        else
            si_->freeState(result);
    }

    siC_->freeControl(control);
}

bool ompl::control::RGRRT::canMoveToward(const Motion *motion, const base::State *target) const
{
    const double nearDistance = si_->distance(motion->state, target);
    for (const auto *state : motion->reachable)
        if (si_->distance(state, target) < nearDistance)
            return true;
    return false;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        computeReachableSet(motion);  // RG-RRT
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;

    while (ptc == false)
    {
        // Sample a random state (with goal biasing)
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Find the closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        // RG-RRT: discard the sample if no reachable state of q_near gets closer to it than q_near itself.
        // This must happen before sampleTo(), which overwrites rstate with the state actually reached.
        if (!canMoveToward(nmotion, rstate))
            continue;

        // Sample a random control that attempts to go towards the random state, and also sample a control duration
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rstate);

        if (cd >= siC_->getMinControlDuration())
        {
            // Create a motion
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, rstate);
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;
            computeReachableSet(motion);  // RG-RRT

            nn_->add(motion);
            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            if (solv)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        // Construct the solution path
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // Set the solution path
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
