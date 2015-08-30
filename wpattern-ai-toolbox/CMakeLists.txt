cmake_minimum_required(VERSION 2.6)
project(lp)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -Wall
    -Wextra
    # Other flags
)

INCLUDE_DIRECTORIES("/usr/include" )

add_executable(pbvi main.cpp)
target_link_libraries(pbvi /usr/lib/lp_solve/liblpsolve55.so /home/branquinho/development/programs/AI-Toolbox/libAIToolboxPOMDP.a /home/branquinho/development/programs/AI-Toolbox/libAIToolboxMDP.a)

install(TARGETS pbvi RUNTIME DESTINATION bin)
