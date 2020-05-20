/*!
\file
\brief File contains ORCAAgent class
*/

#include "Agent.h"

#ifndef ORCA_ORCAAGENT_H
#define ORCA_ORCAAGENT_H


/*!
 * \brief ORCAAgent class implements agent with ORCA algoritm behavior.
 * \details This class describes the interaction interface with the navigation algorithm.
 *  This class implements a collision avoidance algorithm ORCA
 *  the interaction between ORCA and path planner.
 *
 * An illustration of the agent is presented in the figure below.
 *
 * <p align="center">
 * <img width="500" src="./images/reps.png" alt="Agent sheme" >
 * </p>
 *
 * \ingroup ORCAStarLib
 */
class ORCAAgent : public Agent
{

public:
    /*!
     * \brief ORCAAgent default constructor.
     */
    ORCAAgent();

    /*!
     * \brief ORCAAgent constructor with parameters.
     * \param id Agent identifier number for debug and comparison.
     * \param start Start position of agent.
     * \param goal Goal position of agent.
     * \param map Static environment map. It contains information about grid and static obstacles boundaries. More about it see in \ref index "Main page" and \ref Map "Map class".
     * \param options Environment and algorithm options. See \ref EnvironmentOptions "EnvironmentOptions class".
     * \param param agent and algorithm options. See \ref AgentParam "AgentParam class".
     */
    ORCAAgent(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options,
              AgentParam param);

    /*!
     * \brief ORCAAgent copy constructor.
     * \param obj ORCAAgent object to copy.
     */
    ORCAAgent(const ORCAAgent &obj);

    /*!
     * \brief Virtual destructor
     */
    ~ORCAAgent();

    /*!
     * \brief Method for cloning objects. Creates copy of object in memmory and return pointer to copy.
     * \return Pointer to copy of object
     */
    ORCAAgent* Clone() const override;

    /*!
     * \brief Computes new velocity of agent using ORCA algorithm and linear programing.
     */
    void ComputeNewVelocity() override;

    /*!
     * \brief Method for state updating and appling computed velocity.
     */
    void ApplyNewVelocity() override;

    /*!
     * \brief Method for computing preffered velocity and chosing current goal from global path.
     * \return Success of finding new current goal.
     */
    bool UpdatePrefVelocity() override;


    /*!
     * \brief Comparisons operator. Compares id of agents.
     * \param another ORCAAgent to compare.
     * \return equality of this and another identifiers.
     */
    bool operator == (const ORCAAgent &another) const;

    /*!
     * \brief Comparisons operator. Compares id of agents.
     * \param another ORCAAgent to compare.
     * \return Inequality of this and another identifiers.
     */
    bool operator != (const ORCAAgent &another) const;

    /*!
     * \brief Assignment operator.
     * \param obj ORCAAgent to assignment.
     * \return Reference to agent.
     */
    ORCAAgent &operator = (const ORCAAgent &obj);

private:
    //! \cond
    float fakeRadius;
    //! \endcond
};


#endif //ORCA_ORCAAGENT_H
