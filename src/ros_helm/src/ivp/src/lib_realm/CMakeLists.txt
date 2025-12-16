#--------------------------------------------------------
# The CMakeLists.txt for:                       lib_realm
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  InfoCastSettings.cpp
  RealmCast.cpp
  RealmCastSet.cpp
  RealmCastTree.cpp
  RealmRepo.cpp
  RealmSummary.cpp
  WatchCast.cpp
  WatchCluster.cpp
)

SET(HEADERS
  InfoCastSettings.h
  RealmCast.h
  RealmCastSet.h
  RealmCastTree.h
  RealmRepo.h
  RealmSummary.h
  WatchCast.h
  WatchCluster.h
)

# Build Library
ADD_LIBRARY(realm ${SRC})

