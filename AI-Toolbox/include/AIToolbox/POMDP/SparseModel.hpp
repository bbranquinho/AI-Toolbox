#ifndef AI_TOOLBOX_POMDP_SPARSE_MODEL_HEADER_FILE
#define AI_TOOLBOX_POMDP_SPARSE_MODEL_HEADER_FILE

#include <AIToolbox/Utils.hpp>
#include <AIToolbox/MDP/Types.hpp>
#include <AIToolbox/POMDP/Types.hpp>

#include <random>
#include <AIToolbox/Impl/Seeder.hpp>
#include <AIToolbox/ProbabilityUtils.hpp>

namespace AIToolbox {
    namespace POMDP {

#ifndef DOXYGEN_SKIP
        // This is done to avoid bringing around the enable_if everywhere.
        template <typename M, typename = typename std::enable_if<MDP::is_model<M>::value>::type>
        class SparseModel;
#endif

        // Declaration to warn the compiler that this is a template function
        template <typename M, typename = typename std::enable_if<MDP::is_model<M>::value>::type>
        std::istream& operator>>(std::istream &is, SparseModel<M> & m);

        /**
         * @brief This class represents a Partially Observable Markov Decision Process.
         *
         * This class inherits from any valid MDP model type, so that it can
         * use its base methods, and it builds from those. Templated inheritance
         * was chosen to improve performance and keep code small, instead of
         * doing composition.
         *
         * A POMDP is an MDP where the agent, at each timestep, does not know
         * in which state it is. Instead, after each action is performed, it
         * obtains an "observation", which offers some information as to which
         * new state the agent has transitioned to. This observation is
         * determined by an "observation function", that maps S'xAxO to a
         * probability: the probability of obtaining observation O after taking
         * action A and *landing* in state S'.
         *
         * Since now its knowledge is imperfect, in order to represent the
         * knowledge of the state it is currently in, the agent is thus forced
         * to use Beliefs: probability distributions over states.
         *
         * The way a Belief works is that, after each action and observation,
         * the agent can reason as follows: given my previous Belief
         * (distribution over states) that I think I was in, what is now the
         * probability that I transitioned to any particular state? This new
         * Belief can be computed from the Model, given that the agent knows
         * the distributions of the transition and observation functions.
         *
         * Turns out that a POMDP can be viewed as an MDP with an infinite
         * number of states, where each state is essentially a Belief. Since a
         * Belief is a vector of real numbers, there are infinite of them, thus
         * the infinite number of states. While POMDPs can be much more
         * powerful than MDPs for modeling real world problems, where
         * information is usually not perfect, it turns out that this
         * infinite-state property makes them so much harder to solve
         * perfectly, and their solutions much more complex.
         *
         * A POMDP solution is composed by several policies, which apply in
         * different ranges of the Belief space, and suggest different actions
         * depending on the observations received by the agent at each
         * timestep. The values of those policies can be, in the same way,
         * represented as a number of value vectors (called alpha vectors in
         * the literature) that apply in those same ranges of the Belief space.
         * Each alpha vector is somewhat similar to an MDP ValueFunction.
         *
         * @tparam M The particular MDP type that we want to extend.
         */
        template <typename M>
        class SparseModel<M> : public M {
            public:
                using ObservationTable = SparseMatrix3D;

                /**
                 * @brief Basic constructor.
                 *
                 * This constructor initializes the observation function
                 * so that all actions will return observation 0.
                 *
                 * @tparam Args All types of the parent constructor arguments.
                 * @param o The number of possible observations the agent could make.
                 * @param parameters All arguments needed to build the parent Model.
                 */
                template <typename... Args>
                SparseModel(size_t o, Args&&... parameters);

                /**
                 * @brief Basic constructor.
                 *
                 * This constructor takes an arbitrary three dimensional
                 * container and tries to copy its contents into the
                 * observations table.
                 *
                 * The container needs to support data access through
                 * operator[]. In addition, the dimensions of the
                 * container must match the ones provided as arguments
                 * both directly (o) and indirectly (s,a), in the order
                 * s, a, o.
                 *
                 * This is important, as this constructor DOES NOT perform
                 * any size checks on the external containers.
                 *
                 * Internal values of the containers will be converted to double,
                 * so these conversions must be possible.
                 *
                 * In addition, the observation container must contain a
                 * valid transition function.
                 * \sa transitionCheck()
                 *
                 * \sa copyTable3D()
                 *
                 * @tparam ObFun The external observations container type.
                 * @param o The number of possible observations the agent could make.
                 * @param of The observation probability table.
                 * @param parameters All arguments needed to build the parent Model.
                 */
                // Check that ObFun is a triple-table, otherwise we'll call the other constructor!
                template <typename ObFun, typename... Args, typename = typename std::enable_if<std::is_constructible<double,decltype(std::declval<ObFun>()[0][0][0])>::value>::type>
                SparseModel(size_t o, ObFun && of, Args&&... parameters);

                /**
                 * @brief Copy constructor from any valid POMDP model.
                 *
                 * This allows to copy from any other model. A nice use for this is to
                 * convert any model which computes probabilities on the fly into an
                 * POMDP::Model where probabilities are all stored for fast access. Of
                 * course such a solution can be done only when the number of states,
                 * actions and observations is not too big.
                 *
                 * Of course this constructor is available only if the underlying Model
                 * allows to be constructed too.
                 *
                 * @tparam PM The type of the other model.
                 * @param model The model that needs to be copied.
                 */
                template <typename PM, typename = typename std::enable_if<is_model<PM>::value && std::is_constructible<M,PM>::value, int>::type>
                SparseModel(const PM& model);

                /**
                 * @brief This function replaces the Model observation function with the one provided.
                 *
                 * The container needs to support data access through
                 * operator[]. In addition, the dimensions of the
                 * containers must match the ones provided as arguments
                 * (for three dimensions: s,a,o, in this order).
                 *
                 * This is important, as this function DOES NOT perform
                 * any size checks on the external containers.
                 *
                 * Internal values of the container will be converted to double,
                 * so these conversions must be possible.
                 *
                 * @tparam ObFun The external observations container type.
                 * @param of The external observations container.
                 */
                template <typename ObFun>
                void setObservationFunction(const ObFun & of);

                /**
                 * @brief This function samples the POMDP for the specified state action pair.
                 *
                 * This function samples the model for simulated experience. The
                 * transition, observation and reward functions are used to
                 * produce, from the state action pair inserted as arguments, a
                 * possible new state with respective observation and reward.
                 * The new state is picked from all possible states that the
                 * MDP allows transitioning to, each with probability equal to
                 * the same probability of the transition in the model. After a
                 * new state is picked, an observation is sampled from the
                 * observation function distribution, and finally the reward is
                 * the corresponding reward contained in the reward function.
                 *
                 * @param s The state that needs to be sampled.
                 * @param a The action that needs to be sampled.
                 *
                 * @return A tuple containing a new state, observation and reward.
                 */
                std::tuple<size_t,size_t, double> sampleSOR(size_t s,size_t a) const;

                /**
                 * @brief This function samples the POMDP for the specified state action pair.
                 *
                 * This function samples the model for simulated experience.
                 * The transition, observation and reward functions are used to
                 * produce, from the state, action and new state inserted as
                 * arguments, a possible new observation and reward. The
                 * observation and rewards are picked so that they are
                 * consistent with the specified new state.
                 *
                 * @param s The state that needs to be sampled.
                 * @param a The action that needs to be sampled.
                 * @param s1 The resulting state of the s,a transition.
                 *
                 * @return A tuple containing a new observation and reward.
                 */
                std::tuple<size_t, double> sampleOR(size_t s,size_t a,size_t s1) const;

                /**
                 * @brief This function returns the stored observation probability for the specified state-action pair.
                 *
                 * @param s1 The final state of the transition.
                 * @param a The action performed in the transition.
                 * @param o The recorded observation for the transition.
                 *
                 * @return The probability of the specified observation.
                 */
                double getObservationProbability(size_t s1, size_t a, size_t o) const;

                /**
                 * @brief This function *computes* the probability of obtaining an observation given an action and an initial belief.
                 *
                 * @param b The initial belief state.
                 * @param a The action performed.
                 * @param o The resulting observation.
                 *
                 * @return The probability of obtaining the specified observation.
                 */
                double getObservationProbability(const Belief & b, size_t o, size_t a) const;

                /**
                 * @brief This function returns the observation function for a given action.
                 *
                 * @param a The action requested.
                 *
                 * @return The observation function for the input action.
                 */
                const SparseMatrix2D & getObservationFunction(size_t a) const;

                /**
                 * @brief This function returns the number of observations possible.
                 *
                 * @return The total number of observations.
                 */
                size_t getO() const;

                /**
                 * @brief This function returns the observation table for inspection.
                 *
                 * @return The rewards table.
                 */
                const ObservationTable & getObservationFunction() const;

            private:
                size_t O;
                ObservationTable observations_;
                // We need this because we don't know if our parent already has one,
                // and we wouldn't know how to access it!
                mutable std::default_random_engine rand_;

                friend std::istream& operator>> <M>(std::istream &is, SparseModel<M> &);
        };

        template <typename M>
        template <typename... Args>
        SparseModel<M>::SparseModel(size_t o, Args&&... params) : M(std::forward<Args>(params)...), O(o), observations_(this->getA(), SparseMatrix2D(this->getS(), O)),
                                                                  rand_(Impl::Seeder::getSeed())
        {
            for ( size_t a = 0; a < this->getA(); ++a )
                for ( size_t s1 = 0; s1 < this->getS(); ++s1 )
                    observations_[a].insert(s1, 0) = 1.0;
        }

        template <typename M>
        template <typename ObFun, typename... Args, typename>
        SparseModel<M>::SparseModel(size_t o, ObFun && of, Args&&... params) : M(std::forward<Args>(params)...), O(o), observations_(this->getA(), SparseMatrix2D(this->getS(), O)),
                                                                   rand_(Impl::Seeder::getSeed())
        {
            setObservationFunction(of);
        }

        template <typename M>
        template <typename PM, typename>
        SparseModel<M>::SparseModel(const PM& model) : M(model), O(model.getO()), observations_(this->getA(), SparseMatrix2D(this->getS(), O)),
                                           rand_(Impl::Seeder::getSeed())
        {
            for ( size_t a = 0; a < this->getA(); ++a )
                for ( size_t s1 = 0; s1 < this->getS(); ++s1 ) {
                    for ( size_t o = 0; o < O; ++o ) {
                        double p = model.getObservationProbability(s1, a, o);
                        if ( checkDifferentSmall( p, 0.0 ) ) observations_[a].insert(s1, o) = p;
                    }
                    if ( ! isProbability(O, observations_[a].row(s1)) ) throw std::invalid_argument("Input observation table does not contain valid probabilities.");
                }
        }

        template <typename M>
        template <typename ObFun>
        void SparseModel<M>::setObservationFunction(const ObFun & of) {
            for ( size_t s1 = 0; s1 < this->getS(); ++s1 )
                for ( size_t a = 0; a < this->getA(); ++a )
                    if ( ! isProbability(O, of[s1][a]) ) throw std::invalid_argument("Input observation table does not contain valid probabilities.");

            for ( size_t s1 = 0; s1 < this->getS(); ++s1 )
                for ( size_t a = 0; a < this->getA(); ++a )
                    for ( size_t o = 0; o < O; ++o ) {
                        double p = of[s1][a][o];
                        if ( checkDifferentSmall( p, 0.0 ) ) observations_[a].insert(s1, o) = p;
                    }
        }

        template <typename M>
        double SparseModel<M>::getObservationProbability(size_t s1, size_t a, size_t o) const {
            return observations_[a].coeff(s1, o);
        }

        template <typename M>
        const SparseMatrix2D & SparseModel<M>::getObservationFunction(size_t a) const {
            return observations_[a];
        }

        template <typename M>
        size_t SparseModel<M>::getO() const {
            return O;
        }

        template <typename M>
        const typename SparseModel<M>::ObservationTable & SparseModel<M>::getObservationFunction() const {
            return observations_;
        }

        template <typename M>
        std::tuple<size_t,size_t, double> SparseModel<M>::sampleSOR(size_t s, size_t a) const {
            size_t s1, o;
            double r;

            std::tie(s1, r) = this->sampleSR(s, a);
            o = sampleProbability(O, observations_[a].row(s1), rand_);

            return std::make_tuple(s1, o, r);
        }

        template <typename M>
        std::tuple<size_t, double> SparseModel<M>::sampleOR(size_t s, size_t a, size_t s1) const {
            size_t o = sampleProbability(O, observations_[a].row(s1), rand_);
            double r = this->getExpectedReward(s, a, s1);
            return std::make_tuple(o, r);
        }
    }
}

#endif
