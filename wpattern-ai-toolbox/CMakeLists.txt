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
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
SET(CMAKE_CXX_FLAGS "-lboost_program_options")

add_executable(wpattern-ai-toolbox main.cpp)

target_link_libraries(wpattern-ai-toolbox
  ${Boost_LIBRARY_DIRS}/libboost_program_options.a
  /usr/lib/lp_solve/liblpsolve55.so
  /home/branquinho/development/programs/AI-Toolbox/libAIToolboxPOMDP.a
  /home/branquinho/development/programs/AI-Toolbox/libAIToolboxMDP.a)

install(TARGETS wpattern-ai-toolbox RUNTIME DESTINATION bin)
