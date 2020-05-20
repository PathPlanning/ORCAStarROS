/*!
\file
\brief File contains Agent class and AgentParam class.
*/

/*!
\defgroup ORCAStarLib ORCAStar Library
\brief  This module contains set of program interfaces for creating decentralized navigation systems
        based on path planning and collision avoidance algorithms.

\details    The library currently supports ORCA and Theta* algorithms.
            It also contains XML-files input/output modules, static environment description module,
            utility data structures and base modules for alternative algorithms implementations of path planning and collision avoidance
*/


#include "Const.h"
#include "PathPlanner.h"
#include "ThetaStar.h"
#include "Geom.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <type_traits>
#include <cstddef>

#ifndef ORCA_AGENT_H
#define ORCA_AGENT_H


/*!
* \brief Class AgentParam contains agent and algoritms parameters
* \ingroup ORCAStarLib
*/

class AgentParam
{
    public:
        /*!
         * \brief AgentParam default constructor.
         * \details All parametes will be initialized with default values from \ref Const.h "Const.h file".
         * An illustration of the agent is presented in the figure below.
         *
         * <p align="center">
         * <img width="500" src="./images/reps.png" alt="Agent sheme" >
         * </p>

         */
        AgentParam() : sightRadius(CN_DEFAULT_RADIUS_OF_SIGHT), timeBoundary(CN_DEFAULT_TIME_BOUNDARY), timeBoundaryObst(CN_DEFAULT_OBS_TIME_BOUNDARY),
            radius(CN_DEFAULT_SIZE), rEps(CN_DEFAULT_REPS), maxSpeed(CN_DEFAULT_MAX_SPEED), agentsMaxNum(CN_DEFAULT_AGENTS_MAX_NUM) {}

        /*!
         * \brief AgentParam constructor with parameters.
         * \param sr Radius in which the agent takes neighbors into account.
         * \param tb Time within which ORCA algorithm ensures collision avoidance with neighbor agents.
         * \param tbo Time within which ORCA algorithm ensures collision avoidance with neighbor obstacles.
         * \param r Size of the agent (radius of the agent).
         * \param reps Buffer size (more about buffer see \ref index "Main page").
         * \param ms Maximum speed of agent.
         * \param amn Number of neighbors, that the agent takes into account.
         */
        AgentParam(float sr, float tb, float tbo, float r, float reps, float ms, int amn) : sightRadius(sr), timeBoundary(tb), timeBoundaryObst(tbo), radius(r), rEps(reps),
            maxSpeed(ms), agentsMaxNum(amn) {}

        /*!
          \brief AgentParam default destructor.
         */
        ~AgentParam() = default;

        float sightRadius;      ///< Radius in which the agent takes neighbors into account.
        float timeBoundary;     ///<Time within which ORCA algorithm ensures collision avoidance with neighbor agents.
        float timeBoundaryObst; ///<Time within which ORCA algorithm ensures collision avoidance with neighbor obstacles.
        float radius;           ///<Size of the agent (radius of the agent).
        float rEps;             ///<Buffer size (more about buffer see \ref index "Main page").
        float maxSpeed;         ///<Maximum speed of agent.
        int agentsMaxNum;       ///<Number of neighbors, that the agent takes into account.
};


/*!
 * \brief Agent class implements base agent interface and behavior.
 * \ingroup ORCAStarLib
 * \details This class describes the interaction interface with the navigation algorithm.
 *  It is in the successor classes of Agent that it is supposed to implement a collision avoidance algorithm and
 *  the interaction between collision avoidance algorithm and path planner.
 *
 * An illustration of the agent is presented in the figure below.
 *
 * <p align="center">
 * <img width="500" src="./images/reps.png" alt="Agent sheme" >
 * </p>
 */
class Agent
{


    public:
        /*!
         * \brief Agent default constructor.
         */
        Agent();

        /*!
         * \brief Agent constructor with parameters.
         * \param id Agent identifier number for debug and comparison.
         * \param start Start position of agent.
         * \param goal Goal position of agent.
         * \param map Static environment map. It contains information about grid and static obstacles boundaries. More about it see in \ref index "Main page" and \ref Map "Map class".
         * \param options Environment and algorithm options. See \ref EnvironmentOptions "EnvironmentOptions class".
         * \param param agent and algorithm options. See \ref AgentParam "AgentParam class".
         */
        Agent(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options, AgentParam param);

        /*!
         * \brief Agent copy constructor.
         * \param obj Agent object to copy.
         */
        Agent(const Agent &obj);

        /*!
         * \brief Virtual destructor
         */
        virtual ~Agent();

        /*!
         * \brief Abstract method for cloning inheritors objects. Implementations of this method should create copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        virtual Agent* Clone() const = 0;

        /*!
         * \brief Abstract method for new agent velocity computation. Implementations of this method should change newV field.
         */
        virtual void ComputeNewVelocity() = 0;


        /*!
         * \brief Abstract method for state updating and appling computed velocity
         */
        virtual void ApplyNewVelocity() = 0;

        /*!
         * \brief Abstract method for computing preffered velocity and chosing current goal. Can be ovveriden.
         * \return Success of finding new current goal.
         */
        virtual bool UpdatePrefVelocity() = 0;


        /*!
         * \brief SetsS current position of agent. Can be ovveriden
         * \param pos New position.
         */
        virtual void SetPosition(const Point &pos);

        /*!
         * \brief Adds new neighbour. Only agents inside sight radius will be adeed. Can be ovveriden
         * \param neighbour Neighboring agent
         * \param distSq Squred euclidian distance betwen agents
         */
        virtual void AddNeighbour(Agent &neighbour, float distSq);

        /*!
         * \brief Checks, what the agent is on finish. To achieve finish agent should be at least in delta distance from goal position.
         * \return The agent achieve his goal position.
         */
        virtual bool isFinished();

        /*!
         * \brief Sets current velocity. Should be used if the new velocity may not coincide with the computed.
         * \param vel New curren velocity.
         */
        virtual void SetVelocity(const Point &vel);

        /*!
         * \brief Starts pathfinding from start to goal position.
         * \return Success of pathfinding.
         */
        bool InitPath();

        /*!
         * \brief Returns identifier of agent.
         * \return Identifier of agent.
         */
        int GetID() const;

        /*!
         * \brief Returns current position of Agent.
         * \return Current position of Agent.
         */
        Point GetPosition() const;

        /*!
         * \brief Returns current velocity of agent.
         * \return Current velocity of agent.
         */
        Point GetVelocity() const;


        /*!
         * \brief Returns radius of agent.
         * \return Radius of agent.
         */
        float GetRadius() const;

        /*!
         * \brief Returns current goal of agent. For debug purposes.
         * \return Current goal of agent.
         */
        Point GetNext() const;

        /*!
         * \brief Returns start of agent. For ROS Simulation purposes.
         * \return Global start of agent.
         */
        Point GetStart() const;

        /*!
         * \brief Returns goal of agent. For ROS Simulation purposes.
         * \return Global goal of agent.
         */
        Point GetGoal() const;

        /*!
         * \brief Returns parameters of agent. For ROS Simulation purposes.
         * \return Parameters of agent.
         */
        AgentParam GetParam() const;

        /*!
         * \brief Returns collisions count. For debug purposes.
         * \return Pair of collisions counters: collisions with agents and collisions with static obstacles.
         */
        std::pair<unsigned int, unsigned int> GetCollision() const;

        /*!
         * \brief Computes neighbouring obstacles.
         */
        void UpdateNeighbourObst();

        /*!
         * \brief Comparisons operator. Compares id of agents.
         * \param another Agnet to compare.
         * \return equality of this and another identifiers.
         */
        bool operator == (const Agent &another) const;

        /*!
         * \brief Comparisons operator. Compares id of agents.
         * \param another Agent to compare.
         * \return Inequality of this and another identifiers.
         */
        bool operator != (const Agent &another) const;

        /*!
         * \brief Assignment operator.
         * \param obj Agent to assignment.
         * \return Reference to agent.
         */
        Agent & operator = (const Agent& obj);

        /*!
         * \brief Set global planner object.
         * \param pl Global planner object. Will be copied. Object should be inheritor of PathPlanner.
         */
        template <class Planner> void SetPlanner(const Planner &pl)
        {
            static_assert(std::is_base_of<PathPlanner, Planner>::value, "Planner should be inheritor of PathPlanner");
            this->planner = new Planner(pl);
        }


    protected:

        //! @cond
        int id;
        Point start;
        Point goal;
        PathPlanner *planner;
        const EnvironmentOptions *options;
        const Map *map;
        std::vector <std::pair<float, Agent*>> Neighbours;
        std::vector <std::pair<float, ObstacleSegment>> NeighboursObst;

        std::vector <Line> ORCALines;

        Point position;
        Velocity prefV;
        Velocity newV;
        Velocity currV;

        AgentParam param;
        float invTimeBoundaryObst;
        float invTimeBoundary;
        float maxSqObstDist;

        unsigned int collisions;
        unsigned int collisionsObst;

        Point nextForLog;
        //! \endcond
};





#endif //ORCA_AGENT_H
