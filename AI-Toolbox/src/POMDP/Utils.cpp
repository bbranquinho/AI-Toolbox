#include <AIToolbox/POMDP/Utils.hpp>

namespace AIToolbox {
    namespace POMDP {

        VEntry makeVEntry(size_t S, size_t a, size_t O) {
            auto values = MDP::Values(S);
            values.fill(0.0);
            return std::make_tuple(values, a, VObs(O, 0));
        }

        bool operator<(const VEntry & lhs, const VEntry & rhs) {
            if (AIToolbox::operator<(std::get<0>(lhs), std::get<0>(rhs))) return true;
            if (AIToolbox::operator<(std::get<0>(rhs), std::get<0>(lhs))) return false;
            if (std::get<1>(lhs) < std::get<1>(rhs)) return true;
            if (std::get<1>(rhs) < std::get<1>(lhs)) return false;
            if (std::get<2>(lhs) < std::get<2>(rhs)) return true;
            return false;
        }

        bool operator>(const VEntry & lhs, const VEntry & rhs) {
            return !(rhs < lhs);
        }

        double weakBoundDistance(const VList & oldV, const VList & newV) {
            // Here we implement a weak bound (can also be seen in Cassandra's code)
            // This is mostly because a strong bound is more costly (it requires performing
            // multiple LPs) and also the code at the moment does not support it cleanly, so
            // I prefer waiting until I have a good implementation of an LP class that hides
            // complexity from here.
            //
            // The logic of the weak bound is the following: the variation between the old
            // VList and the new one is equal to the maximum distance between a ValueFunction
            // in the old VList with its closest match in the new VList. So the farthest from
            // closest.
            //
            // We define distance between two ValueFunctions as the maximum between their
            // element-wise difference.
            if ( !oldV.size() ) return 0.0;

            double distance = 0.0;
            for ( const auto & newVE : newV ) {
                // Initialize closest distance for newVE as infinity
                double closestDistance = std::numeric_limits<double>::infinity();
                for ( const auto & oldVE : oldV ) {
                    // Compute the distance, we pick the max
                    double distance = (std::get<VALUES>(newVE) - std::get<VALUES>(oldVE)).cwiseAbs().maxCoeff();

                    // Keep the closest, we pick the min
                    closestDistance = std::min(closestDistance, distance);
                }
                // Keep the maximum distance between a new VList and its closest old VList
                distance = std::max(distance, closestDistance);
            }
            return distance;
        }
    }
}
