SET(ORCAStar_HEADERS    Agent.h
                        XMLLogger.h
                        Map.h
                        Const.h
                        PathPlanner.h
                        EnvironmentOptions.h
                        Logger.h
                        Reader.h
                        XMLReader.h
                        Geom.h
                        Summary.h
                        ThetaStar.h
                        LineOfSight.h
                        DirectPlanner.h
                        ORCAAgent.h)

SET(ORCAStar_SOURCES    Agent.cpp
                        XMLLogger.cpp
                        Map.cpp
                        XMLReader.cpp
                        ThetaStar.cpp
                        Geom.cpp
                        EnvironmentOptions.cpp
                        DirectPlanner.cpp
                        ORCAAgent.cpp)

add_library(ORCAStarLib ${ORCAStar_HEADERS} ${ORCAStar_SOURCES})
