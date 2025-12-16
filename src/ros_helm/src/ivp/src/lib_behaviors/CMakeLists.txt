#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_behaviors
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  IvPBehavior.cpp
  IvPContactBehavior.cpp
#  IvPTaskBehavior.cpp
  LifeEventHistory.cpp
  LifeEvent.cpp
  ContactState.cpp
  ContactStateSet.cpp
)

SET(HEADERS
  BehaviorReport.h
  BehaviorSet.h
  IvPBehavior.h
  IvPContactBehavior.h
  LifeEventHistory.h
  LifeEvent.h
  ContactStateSet.h
#  IvPTaskBehavior.h
)

# Build Library
# Use a static library here to avoid cyclic shared library dependencies with
# lib_helmivp and lib_behaviors-marine.
ADD_LIBRARY(behaviors STATIC ${SRC})
SET_TARGET_PROPERTIES(behaviors PROPERTIES POSITION_INDEPENDENT_CODE ON)

TARGET_LINK_LIBRARIES(behaviors
  PRIVATE
    helmivp
    bhvutil
    genutil
    mbutil)

