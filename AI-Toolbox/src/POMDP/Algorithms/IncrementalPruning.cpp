#include <AIToolbox/POMDP/Algorithms/IncrementalPruning.hpp>

namespace AIToolbox {
    namespace POMDP {
        IncrementalPruning::IncrementalPruning(unsigned h, double e) : horizon_(h) {
            setEpsilon(e);
        }

        void IncrementalPruning::setHorizon(unsigned h) {
            horizon_ = h;
        }
        void IncrementalPruning::setEpsilon(double e) {
            if ( e < 0.0 ) throw std::invalid_argument("Epsilon must be >= 0");
            epsilon_ = e;
        }

        unsigned IncrementalPruning::getHorizon() const {
            return horizon_;
        }

        double IncrementalPruning::getEpsilon() const {
            return epsilon_;
        }

        VList IncrementalPruning::crossSum(const VList & l1, const VList & l2, size_t a, bool order) {
            VList c;

            if ( ! ( l1.size() && l2.size() ) ) return c;

            // We can get the sizes of the observation vectors
            // outside since all VEntries for our input VLists
            // are guaranteed to be sized equally.
            const auto O1size  = std::get<OBS>(l1[0]).size();
            const auto O2size  = std::get<OBS>(l2[0]).size();
            for ( const auto & v1 : l1 ) {
                auto O1begin = std::begin(std::get<OBS>(v1));
                auto O1end   = std::end  (std::get<OBS>(v1));
                for ( const auto & v2 : l2 ) {
                    auto O2begin = std::begin(std::get<OBS>(v2));
                    auto O2end   = std::end  (std::get<OBS>(v2));
                    // Cross sum
                    auto v = std::get<VALUES>(v1) + std::get<VALUES>(v2);

                    // This step now depends on which order the two lists
                    // are. This function is only used in this class, so we
                    // know that the two lists are "adjacent"; however one
                    // is after the other. `order` tells us which one comes
                    // first, and we join the observation vectors accordingly.
                    VObs obs; obs.reserve(O1size + O2size);
                    if ( order ) {
                        obs.insert(std::end(obs),O1begin, O1end);
                        obs.insert(std::end(obs),O2begin, O2end);
                    } else {
                        obs.insert(std::end(obs),O2begin, O2end);
                        obs.insert(std::end(obs),O1begin, O1end);
                    }
                    c.emplace_back(std::move(v), a, std::move(obs));
                }
            }

            return c;
        }
    }
}
