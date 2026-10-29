///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <filesystem>
#include <iomanip>
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


bool planPendulum(oc::SimpleSetupPtr &ss, int choice)
{
    auto si = ss->getSpaceInformation();
    if (choice == 1)
    {
        auto planner = std::make_shared<oc::RRT>(si);
        planner->setGoalBias(0.1);
        planner->setIntermediateStates(true);
        ss->setPlanner(planner);
    }
    else if (choice == 2)
    {
        auto planner = std::make_shared<oc::KPIECE1>(si);
        planner->setGoalBias(0.1);
        ss->setPlanner(planner);
    }
    else
    {
        std::cerr << "RG-RRT not implemented yet\n";
        return false;
    }

    ss->setup();
    std::cout << "Pendulum: " << ss->getPlanner()->getName() << '\n';
    const auto solved = ss->solve(30.0);
    std::cout << "Exact solution: " << (solved == ob::PlannerStatus::EXACT_SOLUTION ? "yes" : "no")
              << "; " << solved.asString() << "; time: " << ss->getLastPlanComputationTime() << " s\n";
    if (solved != ob::PlannerStatus::EXACT_SOLUTION)
        return false;
    auto &path = ss->getSolutionPath();
    if (!path.check())
    {
        std::cerr << "Solution failed OMPL's control-path validity check\n";
        return false;
    }

    std::error_code error;
    std::filesystem::create_directories("output", error);
    const double torque = ss->getControlSpace()->as<oc::RealVectorControlSpace>()->getBounds().high[0];
    const std::string file = "output/pendulum_" + ss->getPlanner()->getName() + "_tau" +
                             std::to_string(static_cast<int>(torque)) + ".txt";
    std::ofstream out(file);
    if (error || !out)
    {
        std::cerr << "Cannot open " << file << '\n';
        return false;
    }
    const auto states = path.asGeometric();
    out << std::setprecision(17);
    states.printAsMatrix(out);
    out.close();
    if (!out)
    {
        std::cerr << "Could not write " << file << '\n';
        return false;
    }
    std::cout << "Wrote " << states.getStateCount() << " states to " << file << '\n';
    return true;
}

void benchmarkPendulum(oc::SimpleSetupPtr & /* ss */)
{
    // TODO: Exercise 4.
    std::cout << "Benchmarking not implemented yet\n";
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        if (!(std::cin >> choice)) return 1;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        if (!(std::cin >> which)) return 1;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            if (!(std::cin >> planner)) return 1;
        } while (planner < 1 || planner > 3);

        return planPendulum(ss, planner) ? 0 : 1;
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
