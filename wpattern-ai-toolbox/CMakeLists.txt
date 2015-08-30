CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(wpattern-ai-toolbox)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -Wall
    -Wextra
    # Other flags
)

INCLUDE_DIRECTORIES("/usr/include")

# Boost configuration.
FIND_PACKAGE( Boost 1.53 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
SET(CMAKE_CXX_FLAGS "-lboost_program_options")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)
