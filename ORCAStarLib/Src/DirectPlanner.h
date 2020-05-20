/*!
\file
\brief File contains DirectPlanner class.
*/

#include "PathPlanner.h"
#include "Geom.h"


#ifndef ORCA_DIRECTPLANNER_H
#define ORCA_DIRECTPLANNER_H

/*!
 * \brief DirectPlanner class implements simple planner, which creates direct path from start to goal position.
 * \ingroup ORCAStarLib
 */
class DirectPlanner : public PathPlanner
{
    public:
        /*!
         * \brief DirectPlanner constructor with parametes.
         * \param map Static environment map. It contains information about grid and static obstacles boundaries. More about it see in \ref index "Main page" and \ref Map "Map class".
         * \param options Environment and algorithm options. See \ref EnvironmentOptions "EnvironmentOptions class".
         * \param start Start position of agent.
         * \param goal Goal position of agent.
         * \param radius Radius of agent.
         */
        DirectPlanner(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal, const float &radius);

        /*!
         * \brief DirectPlanner copy constructor.
         * \param obj Object to copy.
         */
        DirectPlanner(const DirectPlanner &obj);

        /*!
         * \brief DirectPlanner destructor.
         */
        ~DirectPlanner() override;


        /*!
         * \brief Returns current goal of agent. Always return goal position.
         * \param [in] curr Current position of agent.
         * \param [out] next Current goal of agent.
         */
        bool GetNext(const Point &curr, Point &next) override;

        /*!
         * \brief Nothing.
         * \return Always true.
         */
        bool CreateGlobalPath() override;

        /*!
         * \brief Method for cloning objects. Creates copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        DirectPlanner* Clone() const override;

        /*!
         * \brief Assignment operator.
         * \param obj Object to assign.
         * \return Reference to assigned object.
         */
        DirectPlanner & operator = (const DirectPlanner &obj);


};

#endif //ORCA_DIRECTPLANNER_H
