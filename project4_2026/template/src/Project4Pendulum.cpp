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
        // TODO: The dimension of your projection for the pendulum
        return 2;
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *cs = state->as<ob::CompoundState>();
        projection(0) = cs->as<ob::SO2StateSpace::StateType>(0)->value;
        projection(1) = cs->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
        // TODO: Your projection for the pendulum
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
    auto theta = std::make_shared<ob::SO2StateSpace>(); // andgle that wraps around
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
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(
        odeSolver, [theta](const ob::State *, const oc::Control *, double, ob::State *result)
        { theta->enforceBounds(result->as<ob::CompoundState>()->components[0]); }));

    // Set the propagation step size and control duration
    si->setPropagationStepSize(0.05); // one step is 0.05 secondsof simulated time
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


void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    auto si = ss->getSpaceInformation();
    if (choice == 1)
        ss->setPlanner(std::make_shared<oc::RRT>(si));
    else if (choice == 2)
        ss->setPlanner(std::make_shared<oc::KPIECE1>(si));
    else
    {
        // TODO: ss->setPlanner(std::make_shared<oc::RGRRT>(si));
        std::cerr << "RG-RRT not implemented yet" << std::endl;
        return;
    }

    ss->setup();
    ob::PlannerStatus solved = ss->solve(30.0);

    if (solved != ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "No exact solution found (" << solved.asString() << ")" << std::endl;
        return;
    }

    // Name the output after the planner and torque bound, e.g. pendulum_RRT_tau3.txt
    double torque = ss->getControlSpace()->as<oc::RealVectorControlSpace>()->getBounds().high[0];
    std::string file = "pendulum_" + ss->getPlanner()->getName() + "_tau" +
                       std::to_string(static_cast<int>(torque)) + ".txt";

    oc::PathControl &path = ss->getSolutionPath();
    path.interpolate();
    std::ofstream out(file);
    path.asGeometric().printAsMatrix(out);
    std::cout << "Found solution; wrote " << path.getStateCount() << " states to " << file << std::endl;
}


void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
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

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
