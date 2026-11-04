///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <vector>

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace ompl
{
    namespace control
    {
        /** Reachability-Guided RRT (Shkolnik, Walter, Tedrake, ICRA 2009).

            Based on OMPL's control-based RRT. Every tree node stores an approximation of its reachable set
            R(q): the valid states obtained by applying numControls_ evenly spaced values of the first control
            dimension for reachSteps_ propagation steps. A random sample is only used to expand its nearest
            node q_near if some state in R(q_near) is closer to the sample than q_near itself; otherwise the
            sample is discarded.

            Assumptions: the control space is a bounded RealVectorControlSpace. Only control dimension 0 is
            varied for R(q); all other dimensions are held at the middle of their bounds. */
        class RGRRT : public base::Planner
        {
        public:
            RGRRT(const SpaceInformationPtr &si);

            ~RGRRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

            void getPlannerData(base::PlannerData &data) const override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }

            // RG-RRT: duration of the reachable-set controls, in propagation steps
            void setReachSteps(unsigned int steps)
            {
                reachSteps_ = steps;
            }

            unsigned int getReachSteps() const
            {
                return reachSteps_;
            }

            // RG-RRT: number of evenly spaced controls used to approximate R(q) (at least 2)
            void setNumControls(unsigned int numControls)
            {
                numControls_ = numControls;
            }

            unsigned int getNumControls() const
            {
                return numControls_;
            }

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                base::State *state{nullptr};
                Control *control{nullptr};
                unsigned int steps{0};
                Motion *parent{nullptr};

                // RG-RRT: approximation of the reachable set R(state)
                std::vector<base::State *> reachable;
            };

            void freeMemory();

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            // RG-RRT: fill motion->reachable by propagating numControls_ controls from motion->state
            void computeReachableSet(Motion *motion) const;

            // RG-RRT: true if some state in R(motion) is closer to target than motion->state is
            bool canMoveToward(const Motion *motion, const base::State *target) const;

            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;
            const SpaceInformation *siC_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            double goalBias_{0.05};
            unsigned int reachSteps_{2};
            unsigned int numControls_{11};

            RNG rng_;
            Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace control
}  // namespace ompl

#endif
