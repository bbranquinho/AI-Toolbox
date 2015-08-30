#define BOOST_TEST_MODULE MDP_Model
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <AIToolbox/MDP/IO.hpp>
#include <AIToolbox/MDP/SparseModel.hpp>

#include "CornerProblem.hpp"

#include <fstream>

BOOST_AUTO_TEST_CASE( construction ) {
    const int S = 5, A = 6;

    AIToolbox::MDP::SparseModel m(S, A);

    BOOST_CHECK_EQUAL(m.getS(), S);
    BOOST_CHECK_EQUAL(m.getA(), A);

    BOOST_CHECK_EQUAL(m.getTransitionProbability(0,0,0), 1.0);
    BOOST_CHECK_EQUAL(m.getTransitionProbability(0,1,0), 1.0);
    BOOST_CHECK_EQUAL(m.getTransitionProbability(0,0,1), 0.0);
    BOOST_CHECK_EQUAL(m.getTransitionProbability(0,1,1), 0.0);

    BOOST_CHECK_EQUAL(m.getExpectedReward(0,0,0), 0.0);
}

BOOST_AUTO_TEST_CASE( copy_construction ) {
    using namespace AIToolbox::MDP;

    GridWorld grid(4, 4);

    auto model = makeCornerProblem(grid);
    size_t S = model.getS(), A = model.getA();

    SparseModel copy(model);

    BOOST_CHECK_EQUAL(model.getDiscount(), copy.getDiscount());
    BOOST_CHECK_EQUAL(S, copy.getS());
    BOOST_CHECK_EQUAL(A, copy.getA());

    for ( size_t s = 0; s < S; ++s ) {
        for ( size_t a = 0; a < A; ++a ) {
            for ( size_t s1 = 0; s1 < S; ++s1 ) {
                BOOST_CHECK_EQUAL(model.getTransitionProbability(s, a, s1), copy.getTransitionProbability(s, a, s1));
                BOOST_CHECK_EQUAL(model.getExpectedReward(s, a, s1), copy.getExpectedReward(s, a, s1));
            }
        }
    }
}

int generator() {
    static int counter = 0;
    return ++counter;
}

BOOST_AUTO_TEST_CASE( files ) {
    const int S = 4, A = 2;
    AIToolbox::MDP::SparseModel m(S,A);

    std::string inputFilename  = "./data/mdp_model.txt";
    std::string outputFilename = "./loadedModel.txt";
    {
        std::ifstream inputFile(inputFilename);

        if ( !inputFile ) BOOST_FAIL("Data to perform test could not be loaded: " + inputFilename);
        BOOST_CHECK( inputFile >> m );
    }
    {
        std::ofstream outputFile(outputFilename);
        if ( !outputFile ) BOOST_FAIL("Could not open file for writing: " + outputFilename);
        BOOST_CHECK( outputFile << m );
    }
    {
        std::ifstream inputFile(inputFilename);
        std::ifstream writtenFile(outputFilename);

        double input, written;
        while ( inputFile >> input ) {
            BOOST_CHECK( writtenFile >> written );
            BOOST_CHECK_EQUAL( written, input );
        }
        BOOST_CHECK( ! ( writtenFile >> written ) );
    }
    // Cleanup
    {
        std::remove(outputFilename.c_str());
    }
}
