///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cmath>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include "CollisionChecking.h"
#include "RG-RRT.h"

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

bool planCar(oc::SimpleSetupPtr &ss, int choice)
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
    std::cout << "Car: " << ss->getPlanner()->getName() << '\n';
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
    const std::string file = "output/car_" + ss->getPlanner()->getName() + ".txt";
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

void benchmarkCar(oc::SimpleSetupPtr & /* ss */)
{
    // TODO: Exercise 4.
    std::cout << "Benchmarking not implemented yet\n";
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        if (!(std::cin >> choice)) return 1;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        if (planner != 3)
        {
            std::error_code error;
            std::filesystem::create_directories("output", error);
            std::ofstream street("output/street.txt");
            if (error || !street) { std::cerr << "Cannot write street.txt\n"; return 1; }
            for (const auto &r : obstacles)
                street << r.x << ' ' << r.y << ' ' << r.width << ' ' << r.height << '\n';
            street.close();
            if (!street) return 1;
        }
        return planCar(ss, planner) ? 0 : 1;
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
