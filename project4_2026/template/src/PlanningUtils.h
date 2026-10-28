#ifndef PROJECT4_PLANNING_UTILS_H
#define PROJECT4_PLANNING_UTILS_H

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/util/RandomNumbers.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace project4
{
namespace ob = ompl::base;
namespace oc = ompl::control;

struct Options
{
    int planner = 0;
    double torque = 3.0;
    double seconds = 30.0;
    unsigned int seed = 0;
    std::string output = "output";
    bool selfTest = false;
};

inline Options parseOptions(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--self-test") { options.selfTest = true; continue; }
        if (i + 1 == argc) throw std::runtime_error("Missing value for " + arg);
        const std::string value = argv[++i];
        if (arg == "--planner")
        {
            if (value == "RRT") options.planner = 1;
            else if (value == "KPIECE1" || value == "KPIECE") options.planner = 2;
            else throw std::runtime_error("Planner must be RRT or KPIECE1");
        }
        else if (arg == "--torque") options.torque = std::stod(value);
        else if (arg == "--time") options.seconds = std::stod(value);
        else if (arg == "--seed") options.seed = static_cast<unsigned int>(std::stoul(value));
        else if (arg == "--output") options.output = value;
        else throw std::runtime_error("Unknown option: " + arg);
    }
    if (!std::isfinite(options.seconds) || options.seconds <= 0)
        throw std::runtime_error("Planning time must be finite and positive");
    if (options.torque != 3 && options.torque != 5 && options.torque != 10)
        throw std::runtime_error("Torque must be 3, 5, or 10");
    if (options.seed) ompl::RNG::setSeed(options.seed);
    return options;
}

inline int menu(const std::string &prompt, int maximum)
{
    int choice;
    while (true)
    {
        std::cout << prompt << std::flush;
        if (!(std::cin >> choice)) throw std::runtime_error("Expected a numeric menu choice");
        if (choice >= 1 && choice <= maximum) return choice;
    }
}

inline void require(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error(message);
}

// Check the actual control path before exporting. Replay at 5 ms intervals,
// finer than the planner's validity sampling, without clamping velocities.
inline double validatePath(oc::SimpleSetup &ss)
{
    auto &path = ss.getSolutionPath();
    auto si = ss.getSpaceInformation();
    require(path.getStateCount() > 1 && path.getControlCount() + 1 == path.getStateCount(),
            "Malformed solution path");
    require(path.check(), "OMPL rejected the control path");
    require(si->distance(path.getState(0), ss.getProblemDefinition()->getStartState(0)) < 1e-9,
            "Path does not begin at the start");
    require(ss.getGoal()->isSatisfied(path.getState(path.getStateCount() - 1)),
            "Path does not reach the goal region");
    const auto &bounds = ss.getControlSpace()->as<oc::RealVectorControlSpace>()->getBounds();
    ob::ScopedState<> replay(ss.getStateSpace());
    double maxError = 0;
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
        require(si->isValid(path.getState(i)), "Invalid solution state");
    for (std::size_t i = 0; i < path.getControlCount(); ++i)
    {
        const auto *u = path.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;
        for (std::size_t j = 0; j < bounds.low.size(); ++j)
            require(std::isfinite(u[j]) && u[j] >= bounds.low[j] && u[j] <= bounds.high[j],
                    "Control outside physical bounds");
        const double duration = path.getControlDuration(i);
        require(std::isfinite(duration) && duration > 0, "Invalid control duration");
        const unsigned int steps = static_cast<unsigned int>(std::ceil(duration / 0.005));
        for (unsigned int k = 1; k <= steps; ++k)
        {
            si->getStatePropagator()->propagate(path.getState(i), path.getControl(i),
                                              duration * k / steps, replay.get());
            require(si->isValid(replay.get()), "Invalid state during dense control replay");
        }
        maxError = std::max(maxError, si->distance(replay.get(), path.getState(i + 1)));
    }
    require(maxError < 1e-5, "Control replay disagrees with solution states");
    return maxError;
}

inline bool plan(oc::SimpleSetupPtr &ss, int choice, const Options &options,
                 const std::string &system, const std::string &suffix = "")
{
    if (choice == 1)
    {
        auto planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        planner->setGoalBias(0.1);
        planner->setIntermediateStates(true);
        ss->setPlanner(planner);
    }
    else if (choice == 2)
    {
        auto planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        planner->setGoalBias(0.1);
        ss->setPlanner(planner);
    }
    else { std::cerr << "RG-RRT is outside Exercises 1 and 2; not implemented.\n"; return false; }
    ss->setup();
    std::cout << "System: " << system << "; planner: " << ss->getPlanner()->getName()
              << suffix << "; seed: " << ompl::RNG::getSeed() << '\n';
    const auto status = ss->solve(options.seconds);
    std::cout << "Exact solution: " << (status == ob::PlannerStatus::EXACT_SOLUTION ? "yes" : "no")
              << "; status: " << status.asString() << "; computation: "
              << ss->getLastPlanComputationTime() << " s\n";
    if (status != ob::PlannerStatus::EXACT_SOLUTION) return false;
    const double error = validatePath(*ss);
    auto &path = ss->getSolutionPath();
    const auto geometric = path.asGeometric();
    const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto dir = std::filesystem::path(options.output) / "trajectories";
    std::filesystem::create_directories(dir);
    const auto base = dir / (system + "_" + ss->getPlanner()->getName() + suffix + "_seed" +
                             std::to_string(ompl::RNG::getSeed()) + "_" + std::to_string(stamp));
    std::ofstream states(base.string() + ".txt"), controls(base.string() + "_controls.txt"),
                  metadata(base.string() + ".json");
    require(states.good() && controls.good() && metadata.good(), "Cannot open trajectory output");
    states << std::setprecision(17);
    geometric.printAsMatrix(states);
    controls << std::setprecision(17);
    path.printAsMatrix(controls);
    double distance;
    ss->getGoal()->isSatisfied(path.getState(path.getStateCount() - 1), &distance);
    metadata << std::setprecision(17) << "{\n  \"system\": \"" << system
             << "\",\n  \"planner\": \"" << ss->getPlanner()->getName()
             << "\",\n  \"seed\": " << ompl::RNG::getSeed()
             << ",\n  \"exact\": true,\n  \"seconds\": " << ss->getLastPlanComputationTime()
             << ",\n  \"states\": " << geometric.getStateCount()
             << ",\n  \"duration\": " << path.length()
             << ",\n  \"goal_distance\": " << distance
             << ",\n  \"replay_error\": " << error
             << ",\n  \"torque\": " << options.torque << "\n}\n";
    states.close(); controls.close(); metadata.close();
    require(!states.fail() && !controls.fail() && !metadata.fail(), "Failed writing trajectory output");
    std::cout << "Validated " << geometric.getStateCount() << " states; simulated duration "
              << path.length() << " s; goal distance " << distance << "; replay error " << error
              << "\nExported " << base.string() << ".txt (states), _controls.txt, .json\n";
    return true;
}
} // namespace project4
#endif
