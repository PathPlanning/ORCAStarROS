/*!
\file
\brief File contains main constants.
*/

#ifndef ORCA_CONST_H
#define ORCA_CONST_H


#define Velocity Point              ///< Velocity type definition.
#define Vector Point                ///< Vector type definition.


#define CN_EPS 0.00001f                 ///< Epsilon for float number operations definition
#define CN_PI_CONSTANT 3.14159265359    ///< Pi number operations definition
#define CN_SQRT_TWO    1.41421356237    ///< SQRT(2) number operations definition
#define FULL_OUTPUT    false            ///< Enable full output to std stream


#define CNS_TAG_ROOT                    "root" ///< Enable full output to std stream

#define CNS_TAG_AGENTS                  "agents"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_NUM            "number"    ///< XML tag or attribute
    #define CNS_TAG_ATTR_TYPE           "type"  ///< XML tag or attribute

#define CNS_TAG_DEF_PARAMS              "default_parameters"    ///< XML tag or attribute
    #define CNS_TAG_ATTR_SIZE               "size"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_MAXSPEED           "movespeed" ///< XML tag or attribute
    #define CNS_TAG_ATTR_AGENTSMAXNUM       "agentsmaxnum"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_TIMEBOUNDARY       "timeboundary"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_SIGHTRADIUS        "sightradius"   ///< XML tag or attribute
    #define CNS_TAG_ATTR_TIMEBOUNDARYOBST   "timeboundaryobst"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_REPS               "reps"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_PARACTNUM          "paractivatingnum"  ///< XML tag or attribute

#define CNS_TAG_AGENT                   "agent" ///< XML tag or attribute
    #define CNS_TAG_ATTR_ID             "id"    ///< XML tag or attribute
    #define CNS_TAG_ATTR_STX             "start.xr" ///< XML tag or attribute
    #define CNS_TAG_ATTR_STY             "start.yr" ///< XML tag or attribute
    #define CNS_TAG_ATTR_GX             "goal.xr"   ///< XML tag or attribute
    #define CNS_TAG_ATTR_GY             "goal.yr"   ///< XML tag or attribute

#define CNS_TAG_ALG         "algorithm" ///< XML tag or attribute
    #define CNS_TAG_ST          "searchtype"    ///< XML tag or attribute
    #define CNS_TAG_MT          "metrictype"    ///< XML tag or attribute
    #define CNS_TAG_BT          "breakingties"  ///< XML tag or attribute
    #define CNS_TAG_AS          "allowsqueeze"  ///< XML tag or attribute
    #define CNS_TAG_CC          "cutcorners"    ///< XML tag or attribute
    #define CNS_TAG_HW          "hweight"   ///< XML tag or attribute
    #define CNS_TAG_TS          "timestep"  ///< XML tag or attribute
    #define CNS_TAG_DEL         "delta" ///< XML tag or attribute

#define CNS_TAG_MAP         "map"   ///< XML tag or attribute
    #define CNS_TAG_CELLSIZE    "cellsize"  ///< XML tag or attribute
    #define CNS_TAG_WIDTH       "width" ///< XML tag or attribute
    #define CNS_TAG_HEIGHT      "height"    ///< XML tag or attribute
    #define CNS_TAG_GRID        "grid"  ///< XML tag or attribute
    #define CNS_TAG_ROW         "row"   ///< XML tag or attribute

#define CNS_TAG_OBSTS       "obstacles" ///< XML tag or attribute
    #define CNS_TAG_OBST        "obstacle"  ///< XML tag or attribute
    #define CNS_TAG_VERTEX      "vertex"    ///< XML tag or attribute
    #define CNS_TAG_ATTR_X      "xr"    ///< XML tag or attribute
    #define CNS_TAG_ATTR_Y      "yr"    ///< XML tag or attribute

#define CNS_TAG_LOG         "log"   ///< XML tag or attribute
    #define CNS_TAG_SUM             "summary"   ///< XML tag or attribute
    #define CNS_TAG_ATTR_SR         "successrate"   ///< XML tag or attribute
    #define CNS_TAG_ATTR_RUNTIME    "runtime"   ///< XML tag or attribute
    #define CNS_TAG_ATTR_FLOWTIME   "flowtime"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_MAKESPAN   "makespan"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_COL_AGNT   "collisions"    ///< XML tag or attribute
    #define CNS_TAG_ATTR_COL_OBST   "collisionsobst"    ///< XML tag or attribute
    #define CNS_TAG_PATH            "path"  ///< XML tag or attribute
    #define CNS_TAG_ATTR_PATHFOUND  "pathfound" ///< XML tag or attribute
    #define CNS_TAG_ATTR_STEPS      "steps" ///< XML tag or attribute
    #define CNS_TAG_STEP            "step"  ///< XML tag or attribute


//XML tags' attributes
#define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps" ///< XML tag or attribute
#define CNS_TAG_ATTR_NODESCREATED   "nodescreated"  ///< XML tag or attribute
#define CNS_TAG_ATTR_LENGTH         "length"        ///< XML tag or attribute
#define CNS_TAG_ATTR_LENGTH_SCALED  "length_scaled" ///< XML tag or attribute
#define CNS_TAG_ATTR_TIME           "time"  ///< XML tag or attribute






#define CNS_SP_ST_THETA     "thetastar" ///< XML text constant
#define CNS_SP_ST_DIR       "direct"    ///< XML text constant

#define CNS_AT_ST_ORCA      "orca"  ///< XML text constant

#define CNS_SP_MT_DIAG      "diagonal"  ///< XML text constant
#define CNS_SP_MT_MANH      "manhattan" ///< XML text constant
#define CNS_SP_MT_EUCL      "euclid"    ///< XML text constant
#define CNS_SP_MT_CHEB      "chebyshev" ///< XML text constant

#define CN_SP_MT_DIAG   0   ///< Diagonal distance constant
#define CN_SP_MT_MANH   1   ///< Manhattan distance constant
#define CN_SP_MT_EUCL   2   ///< Euclid distance constant
#define CN_SP_MT_CHEB   3   ///< Chebyshev distance constant

#define CN_SP_ST_THETA  0   ///< Theta* path planning algorithm constant
#define CN_SP_ST_DIR    1   ///< Direct planning algorithm constant




// Default values
#define CN_DEFAULT_SIZE 1                           ///< Default agent size value
#define CN_DEFAULT_MAX_SPEED 1                      ///< Default agent max speed value
#define CN_DEFAULT_AGENTS_MAX_NUM 10                ///< Default agent max neighbours number value
#define CN_DEFAULT_TIME_BOUNDARY 5.4                ///< Default ORCA time boundary value
#define CN_DEFAULT_OBS_TIME_BOUNDARY 10             ///< Default ORCA time boundary for obstacles value
#define CN_DEFAULT_RADIUS_OF_SIGHT 10               ///< Default agent radius of sight value
#define CN_DEFAULT_METRIC_TYPE CN_SP_MT_EUCL        ///< Default Theta* heuristic-function value
#define CN_DEFAULT_BREAKINGTIES 0                   ///< Default Theta* BREAKINGTIES value
#define CN_DEFAULT_ALLOWSQUEEZE 0                   ///< Default Theta* ALLOWSQUEEZE value
#define CN_DEFAULT_CUTCORNERS 0                     ///< Default Theta* CUTCORNERS value
#define CN_DEFAULT_HWEIGHT 1                        ///< Default H-weight for Theta* value
#define CN_DEFAULT_TIME_STEP 0.25                   ///< Default simulation timestep value
#define CN_DEFAULT_DELTA 0.1                        ///< Default delta value
#define CN_DEFAULT_REPS 0.1                        ///< Default agent buffer size value


#define CN_DEFAULT_ST CN_SP_ST_THETA                ///< Default path planning algorithm
#define CNS_DEFAULT_ST CNS_SP_ST_THETA              ///< Default path planning algorithm
#define CNS_DEFAULT_AGENT_TYPE CNS_AT_ST_ORCA       ///< Default collision avoidance algorithm





//Search Parameters
#define CN_SP_BT_GMIN 0 ///< Gmin constant
#define CN_SP_BT_GMAX 1 ///< Gmax constant

//Grid Cell
#define CN_GC_NOOBS 0 ///< Not obstacle cell in XML
#define CN_GC_OBS   1 ///< Obstacle cell in XML





#endif //ORCA_CONST_H
