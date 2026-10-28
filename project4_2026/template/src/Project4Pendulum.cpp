///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"
#include "PlanningUtils.h"
#include <ompl/control/SimpleDirectedControlSampler.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void defaultCellSizes() override
    {
        cellSizes_ = {0.2, 0.5};
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *cs = state->as<ob::CompoundState>();
        projection(0) = cs->as<ob::SO2StateSpace::StateType>(0)->value;
        projection(1) = cs->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType & q , const ompl::control::Control * control ,
                 ompl::control::ODESolver::StateType & qdot )
{
    // torque
    double tau = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
    qdot.resize(q.size(), 0);
    qdot[0] = q[1];
    qdot[1] = -9.81 * std::cos(q[0]) + tau;
}
ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // State space: theta in SO2 (wraps at +-pi), omega in [-10, 10]
    auto theta = std::make_shared<ob::SO2StateSpace>(); // angle that wraps around
    auto omega = std::make_shared<ob::RealVectorStateSpace>(1); // real vector for angular velocity
    omega->setBounds(-10.0, 10.0);

    auto space = std::make_shared<ob::CompoundStateSpace>(); // state = (theta, omega)
    space->addSubspace(theta, 1.0);
    space->addSubspace(omega, 0.5);
    space->lock();
    space->registerDefaultProjection(std::make_shared<PendulumProjection>(space.get()));

    // Control space: tau in [-torque, torque]
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 1);
    ob::RealVectorBounds cbounds(1);
    // torque limits
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);

    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    oc::SpaceInformation *si = ss->getSpaceInformation().get();

    // No obstacles; only the omega bound must hold
    ss->setStateValidityChecker([si](const ob::State *state) { return si->satisfiesBounds(state); });

    // Integrate pendulumODE, then wrap theta back into [-pi, pi)
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE, 0.005);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(
        odeSolver, [theta](const ob::State *, const oc::Control *, double, ob::State *result)
        { theta->enforceBounds(result->as<ob::CompoundState>()->components[0]); }));

    si->setDirectedControlSamplerAllocator([](const oc::SpaceInformation *info)
        { return std::make_shared<oc::SimpleDirectedControlSampler>(info, 5); });

    // Set the propagation step size and control duration
    si->setPropagationStepSize(0.05); // one step is 0.05 seconds of simulated time
    si->setMinMaxControlDuration(1, 10); // We hold the sampled control for 1 to 10 steps (0.05 to 0.5 seconds)

    // Hanging down at rest -> pointing up at rest
    const double halfPi = std::acos(0.0);  // pi / 2
    ob::ScopedState<> start(space), goal(space);
    start[0] = -halfPi;
    start[1] = 0.0;
    goal[0] = halfPi;
    goal[1] = 0.0;
    ss->setStartAndGoalStates(start, goal, 0.1);

    return ss;
}


bool planPendulum(oc::SimpleSetupPtr &ss, int choice, const project4::Options &options)
{
    return project4::plan(ss, choice, options, "pendulum",
                          "_tau" + std::to_string(static_cast<int>(options.torque)));
}

void benchmarkPendulum(oc::SimpleSetupPtr & /* ss */)
{
    std::cout << "Benchmarking is outside Exercises 1 and 2; not implemented.\n";
}

void testPendulum()
{
    auto ss = createPendulum(3);
    ss->setup();
    ob::ScopedState<> state(ss->getStateSpace());
    state[0] = -std::acos(0.0); state[1] = 0;
    project4::require(ss->getSpaceInformation()->isValid(state.get()), "Valid pendulum rejected");
    for (double omega : {-10.01, 10.01})
    {
        state[1] = omega;
        project4::require(!ss->getSpaceInformation()->isValid(state.get()), "Invalid omega accepted");
    }
    state[0] = 0.4; state[1] = -2;
    Eigen::VectorXd projection(2);
    ss->getStateSpace()->getDefaultProjection()->project(state.get(), projection);
    project4::require(std::abs(projection[0] - 0.4) < 1e-12 && projection[1] == -2,
                      "Pendulum projection incorrect");
    auto si = ss->getSpaceInformation();
    auto *control = si->allocControl();
    control->as<oc::RealVectorControlSpace::ControlType>()->values[0] = 3;
    oc::ODESolver::StateType derivative;
    pendulumODE({0, 2}, control, derivative);
    project4::require(derivative[0] == 2 && std::abs(derivative[1] + 6.81) < 1e-12,
                      "Pendulum ODE incorrect");
    state[0] = std::acos(-1.0) - 0.001; state[1] = 2;
    ob::ScopedState<> result(ss->getStateSpace());
    si->getStatePropagator()->propagate(state.get(), control, 0.05, result.get());
    project4::require(result[0] < 0 && si->isValid(result.get()), "Angle wrapping failed");
    si->freeControl(control);
    std::cout << "Pendulum self-test passed: ODE, bounds rejection, projection, propagation/wrapping.\n";
}

int main(int argc, char **argv)
{
    try
    {
        auto options = project4::parseOptions(argc, argv);
        if (options.selfTest) { testPendulum(); return 0; }
        int action = 1;
        if (!options.planner)
        {
            action = project4::menu("Plan or Benchmark?\n (1) Plan\n (2) Benchmark\n", 2);
            const int torque = project4::menu("Torque?\n (1) 3\n (2) 5\n (3) 10\n", 3);
            options.torque = torque == 1 ? 3 : torque == 2 ? 5 : 10;
        }
        auto ss = createPendulum(options.torque);
        if (action == 2) { benchmarkPendulum(ss); return 0; }
        if (!options.planner)
            options.planner = project4::menu("What Planner?\n (1) RRT\n (2) KPIECE1\n (3) RG-RRT\n", 3);
        return planPendulum(ss, options.planner, options) ? 0 : 1;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
