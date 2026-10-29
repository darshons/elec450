///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Darshon Singh (ds161), Winson Lin (wl85)
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
        // Reachability-guided RRT (Shkolnik, Walter and Tedrake, ICRA 2009), built from OMPL's control RRT.
        // Every tree node stores R(q): the valid states reached by applying a fixed set of controls for a
        // fixed duration. A random sample is used only if some state in R(q_near) is closer to it than
        // q_near itself; otherwise it is discarded and a new sample is drawn.
        // Assumes a bounded RealVectorControlSpace. The reachable-set controls are evenly spaced over the
        // bounds of control dimension 0; all other control dimensions are held at the centre of their bounds.
        class RGRRT : public base::Planner
        {
        public:
            RGRRT(const SpaceInformationPtr &si);
            ~RGRRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;

            void setGoalBias(double goalBias) { goalBias_ = goalBias; }
            double getGoalBias() const { return goalBias_; }

            // Number of evenly spaced controls used to approximate R(q) (at least 2).
            void setReachableControlCount(unsigned int count) { reachableControls_ = count; }
            unsigned int getReachableControlCount() const { return reachableControls_; }

            // Duration of each reachable-set control, in propagation steps.
            void setReachableSteps(unsigned int steps) { reachableSteps_ = steps; }
            unsigned int getReachableSteps() const { return reachableSteps_; }

        protected:
            class Motion
            {
            public:
                Motion() = default;
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()) {}

                base::State *state{nullptr};
                Control *control{nullptr};
                unsigned int steps{0};
                Motion *parent{nullptr};
                // Approximation of the reachable set R(state); only valid states are stored.
                std::vector<base::State *> reachable;
            };

            void freeMemory();
            void computeReachable(Motion *motion);
            // True when some state of R(motion) is closer to the sample than motion itself.
            bool reachableTowards(const Motion *motion, const base::State *sample) const;

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;
            const SpaceInformation *siC_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            // Controls applied to compute every reachable set, allocated in setup().
            std::vector<Control *> reachableControlSet_;

            double goalBias_{0.05};
            unsigned int reachableControls_{11};
            unsigned int reachableSteps_{5};
            RNG rng_;
            Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace control
}  // namespace ompl

#endif
