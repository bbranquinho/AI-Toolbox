#include <AIToolbox/MDP/Model.hpp>

#include <AIToolbox/Impl/Seeder.hpp>
#include <AIToolbox/ProbabilityUtils.hpp>

namespace AIToolbox {
    namespace MDP {
        Model::Model(size_t s, size_t a, double discount) : S(s), A(a), discount_(discount), transitions_(boost::extents[S][A][S]), rewards_(boost::extents[S][A][S]),
                                                       rand_(Impl::Seeder::getSeed())
        {
            // Make transition table true probability
            for ( size_t s = 0; s < S; ++s )
                for ( size_t a = 0; a < A; ++a )
                    transitions_[s][a][s] = 1.0;
        }

        std::pair<size_t, double> Model::sample(size_t s, size_t a) const {
            size_t s1 = sampleProbability(transitions_[s][a], S, rand_);

            return std::make_pair(s1, rewards_[s][a][s1]);
        }

        double Model::getTransitionProbability(size_t s, size_t a, size_t s1) const {
            return transitions_[s][a][s1];
        }

        double Model::getExpectedReward(size_t s, size_t a, size_t s1) const {
            return rewards_[s][a][s1];
        }

        void Model::setDiscount(double d) {
            if ( d <= 0.0 || d > 1.0 ) throw std::invalid_argument("Discount parameter must be in (0,1]");
            discount_ = d;
        }

        size_t Model::getS() const { return S; }
        size_t Model::getA() const { return A; }
        double Model::getDiscount() const { return discount_; }

        const Model::TransitionTable & Model::getTransitionFunction() const { return transitions_; }
        const Model::RewardTable &     Model::getRewardFunction()     const { return rewards_; }
    }
}