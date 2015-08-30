CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

ADD_EXECUTABLE(wpattern-ai-toolbox main.cpp)
TARGET_LINK_LIBRARIES(wpattern-ai-toolbox /usr/lib/lp_solve/liblpsolve55.so /home/branquinho/development/programs/AI-Toolbox/libAIToolboxPOMDP.a /home/branquinho/development/programs/AI-Toolbox/libAIToolboxMDP.a ${Boost_LIBRARIES})

INSTALL(TARGETS wpattern-ai-toolbox RUNTIME DESTINATION bin)
