/*!
\file
\brief File contains PathPlanner class.
*/

#include <list>

#include "EnvironmentOptions.h"
#include "Map.h"


#ifndef ORCA_PATHPLANNER_H
#define ORCA_PATHPLANNER_H



/*!
 * \brief PathPlanner class implements base pathfinder interface and behavior.
 * \details This class describes the interface of the pathfinding algorithm.
 *  It is in the successor classes of PathPlanner that it is supposed to implement a global path planner.
 *
 * \ingroup ORCAStarLib
 */

class PathPlanner
{
    public:
        /*!
         * \brief PathPlanner copy constructor.
         * \param obj Object to copy.
         */
        PathPlanner(const PathPlanner &obj) : map(obj.map), options(obj.options), glStart(obj.glStart), glGoal(obj.glGoal), radius(obj.radius) {}

        /*!
         * \brief PathPlanner constructor with parametes.
         * \param map Static environment map. It contains information about grid and static obstacles boundaries. More about it see in \ref index "Main page" and \ref Map "Map class".
         * \param options Environment and algorithm options. See \ref EnvironmentOptions "EnvironmentOptions class".
         * \param start Start position of agent.
         * \param goal Goal position of agent.
         * \param radius Radius of agent.
         */
        PathPlanner(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal, const float &radius)
            : map(&map), options(&options), glStart(start), glGoal(goal), radius(radius) {};

        /*!
         * \brief Virtual PathPlanner destructor.
         */
        virtual ~PathPlanner() {map = nullptr; options = nullptr;}

        /*!
         * \brief Abstract method for cloning inheritors objects. Implementations of this method should create copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        virtual PathPlanner* Clone() const = 0;

        /*!
         * \brief Abstract method. Should finds path from global start to global goal position.
         * \return Success of pathfinding.
         */
        virtual bool CreateGlobalPath() = 0;

        /*!
         * \brief Abstract method. Should returns current goal of agent.
         * \param [in] curr Current position of agent.
         * \param [out] next Current goal of agent.
         */
        virtual bool GetNext(const Point &curr, Point &next) = 0;


        /*!
         * \brief Assignment operator.
         * \param obj Object to assign.
         * \return Reference to assigned object.
         */
        PathPlanner & operator = (const PathPlanner &obj)
        {map = obj.map; options = obj.options; glStart = obj.glStart; glGoal = obj.glGoal; radius = obj.radius; return *this;}

    protected:
       //!\cond
        const Map *map;
        const EnvironmentOptions *options;
        Point glStart;
        Point glGoal;
        float radius;
        //!\endcond

};

#endif //ORCA_PATHPLANNER_H
