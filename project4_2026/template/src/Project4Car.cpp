///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include "CollisionChecking.h"
#include "RG-RRT.h"
#include "PlanningUtils.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}
    unsigned int getDimension() const override { return 3; }
    void defaultCellSizes() override { cellSizes_ = {0.5, 0.5, 0.5}; }
    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *cs = state->as<ob::CompoundState>();
        const auto *pose = cs->as<ob::SE2StateSpace::StateType>(0);
        projection[0] = pose->getX();
        projection[1] = pose->getY();
        projection[2] = cs->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    }
};

void carODE(const oc::ODESolver::StateType &q, const oc::Control *control,
            oc::ODESolver::StateType &qdot)
{
    const auto *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    qdot.resize(q.size(), 0);
    qdot[0] = q[3] * std::cos(q[2]);
    qdot[1] = q[3] * std::sin(q[2]);
    qdot[2] = u[0];
    qdot[3] = u[1];
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // A 12 x 10 workspace with alternating barriers. The direct start-goal
    // segment y=5 crosses both; go above the first and below the second.
    obstacles = {{3.5, 0.0, 1.0, 6.0}, {7.0, 4.0, 1.0, 6.0}};
}

oc::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    auto pose = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0); bounds.setHigh(0, 12); bounds.setHigh(1, 10);
    pose->setBounds(bounds);
    auto velocity = std::make_shared<ob::RealVectorStateSpace>(1);
    velocity->setBounds(-2, 2); // reverse motion is allowed
    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(pose, 1.0);
    space->addSubspace(velocity, 0.5);
    space->lock();
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));
    auto controls = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(-1.5); controlBounds.setHigh(1.5);
    controls->setBounds(controlBounds); // omega [rad/s], acceleration [units/s^2]
    auto ss = std::make_shared<oc::SimpleSetup>(controls);
    auto *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si, obstacles](const ob::State *state)
    {
        const auto *p = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        return si->satisfiesBounds(state) && isValidPoint(p->getX(), p->getY(), obstacles);
    });
    auto solver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE, 0.005);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(solver,
        [](const ob::State *, const oc::Control *, double, ob::State *result)
        {
            // Wrap only the periodic heading; do not clamp position or speed.
            auto *angle = result->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)
                                ->as<ob::SO2StateSpace::StateType>(1);
            ob::SO2StateSpace().enforceBounds(angle);
        }));
    si->setPropagationStepSize(0.05);
    si->setMinMaxControlDuration(1, 20); // 0.05--1.0 seconds
    si->setDirectedControlSamplerAllocator([](const oc::SpaceInformation *info)
        { return std::make_shared<oc::SimpleDirectedControlSampler>(info, 5); });
    ob::ScopedState<> start(space), goal(space);
    start[0] = 1; start[1] = 5; start[2] = 0; start[3] = 0;
    goal[0] = 11; goal[1] = 5; goal[2] = 0; goal[3] = 0;
    // Compound metric: Euclidean position + 0.5*angle + 0.5*speed error.
    ss->setStartAndGoalStates(start, goal, 0.5);
    return ss;
}

bool planCar(oc::SimpleSetupPtr &ss, int choice, const project4::Options &options)
{
    return project4::plan(ss, choice, options, "car");
}

void benchmarkCar(oc::SimpleSetupPtr & /* ss */)
{
    std::cout << "Benchmarking is outside Exercises 1 and 2; not implemented.\n";
}

void testCar(std::vector<Rectangle> &obstacles)
{
    auto ss = createCar(obstacles);
    ss->setup();
    ob::ScopedState<> state(ss->getStateSpace());
    state[0] = 1; state[1] = 5; state[2] = 0; state[3] = -1;
    auto si = ss->getSpaceInformation();
    project4::require(si->isValid(state.get()), "Reverse motion incorrectly rejected");
    for (double speed : {-2.01, 2.01})
    {
        state[3] = speed;
        project4::require(!si->isValid(state.get()), "Out-of-bounds speed accepted");
    }
    state[3] = 0; state[0] = -0.01;
    project4::require(!si->isValid(state.get()), "Out-of-bounds position accepted");
    for (const auto &r : obstacles)
    {
        state[0] = r.x + r.width / 2; state[1] = r.y + r.height / 2;
        project4::require(!si->isValid(state.get()), "Obstacle collision accepted");
    }
    state[0] = 1.2; state[1] = 2.3; state[2] = 0.4; state[3] = -0.7;
    Eigen::VectorXd projection(3);
    ss->getStateSpace()->getDefaultProjection()->project(state.get(), projection);
    project4::require(projection[0] == 1.2 && projection[1] == 2.3 && projection[2] == -0.7,
                      "Car projection incorrect");
    auto *control = si->allocControl();
    auto *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    u[0] = 0.5; u[1] = -1;
    oc::ODESolver::StateType derivative;
    carODE({1, 2, 0, -2}, control, derivative);
    project4::require(derivative == oc::ODESolver::StateType({-2, 0, 0.5, -1}), "Car ODE incorrect");
    state[0] = 1; state[1] = 2; state[2] = 0; state[3] = 0;
    u[0] = 0; u[1] = 1;
    ob::ScopedState<> result(ss->getStateSpace());
    si->getStatePropagator()->propagate(state.get(), control, 0.5, result.get());
    project4::require(std::abs(result[0] - 1.125) < 1e-9 && std::abs(result[3] - 0.5) < 1e-9,
                      "Car integration incorrect");
    si->freeControl(control);
    std::cout << "Car self-test passed: ODE, reverse, bounds/collision rejection, projection, integration.\n";
}

int main(int argc, char **argv)
{
    try
    {
        const auto options = project4::parseOptions(argc, argv);
        std::vector<Rectangle> obstacles;
        makeStreet(obstacles);
        if (options.selfTest) { testCar(obstacles); return 0; }
        int action = 1, planner = options.planner;
        if (!planner) action = project4::menu("Plan or Benchmark?\n (1) Plan\n (2) Benchmark\n", 2);
        auto ss = createCar(obstacles);
        if (action == 2) { benchmarkCar(ss); return 0; }
        if (!planner)
            planner = project4::menu("What Planner?\n (1) RRT\n (2) KPIECE1\n (3) RG-RRT\n", 3);
        // Export the same obstacle data used by the validity checker.
        std::filesystem::create_directories(options.output);
        std::ofstream street(std::filesystem::path(options.output) / "street.txt");
        project4::require(street.good(), "Cannot write street.txt");
        street << std::setprecision(17);
        for (const auto &r : obstacles) street << r.x << ' ' << r.y << ' ' << r.width << ' ' << r.height << '\n';
        street.close();
        project4::require(!street.fail(), "Failed writing street.txt");
        return planCar(ss, planner, options) ? 0 : 1;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
