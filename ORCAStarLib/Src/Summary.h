/*!
\file
\brief File contains Summary class.
*/


#include <ostream>
#include <string>

#ifndef ORCA_SUMMARY_H
#define ORCA_SUMMARY_H

/*!
* \brief Class Summary contains brief information about execution results.
*
* \details Summary includes following values:
* * `Success Rate` — shows the percent of agents, which succeed their tasks;
* * `Runtime` — shows the time of running of task;
* * `Flowtime` — shows the sum of steps of all agents;
* * `Makespan` — shows the maximum value of steps of amoung all agents;
* * `Collisions (agents)` — shows the number of collisions between agents while execution of task;
* * `Collisions (obstacles)` — shows the number of collisions between agents and static obstacles while execution of task;
*
* An illustration of the flowtime and makespan is presented in the figure below.
*
* <p align="center">
* <img width="300" src="./images/ftms.png" alt="Agent sheme" >
* </p>
*
* \ingroup ORCAStarLib
*/
class Summary
{
    public:
       /*!
        * \brief Default constructor.
        */
        Summary() = default;
        /*!
         * \brief Default destructor.
         */
        ~Summary() = default;
        /*!
         * \brief Constructor with parameters.
         * \param srate The percent of agents, which succsed their tasks.
         * \param runtime The time of running of a task.
         * \param flowtime The sum of time steps of all agents.
         * \param makespan The maximum value of time steps of amoung all agents.
         * \param collisions The number of collisions between agents while execution of task.
         * \param collisionsObst The number of collisions between agents and static obstacles while execution of task.
         */
        Summary(float srate, float runtime, float flowtime, float makespan, int collisions, int collisionsObst)
            : successRate(srate), runTime(runtime), flowTime(flowtime), makeSpan(makespan), collisions(collisions), collisionsObst(collisionsObst){}

        /*!
         * \brief Copy constructor.
         * \param obj Object to copy.
         */
        Summary(const Summary &obj)
            : successRate(obj.successRate), runTime(obj.runTime), flowTime(obj.flowTime), makeSpan(obj.makeSpan), collisions(obj.collisions), collisionsObst(obj.collisionsObst){}

        /*!
         * \brief Creates STL string, which contains all values splitted with tabulation.
         * \return STL string, which contains all values splitted with tabulation.
         */
        std::string ToString() const
        {
            return std::to_string(successRate) + "\t"
                 + std::to_string(runTime) + "\t"
                 + std::to_string(makeSpan) + "\t"
                 + std::to_string(flowTime) + "\t"
                 + std::to_string(collisions) + "\t"
                 + std::to_string(collisionsObst) + "\n";
        }


        float successRate;  ///< Shows the percent of agents, which succsed their tasks.
        float runTime;      ///< Shows the time of running of task.
        float flowTime;     ///< Shows the sum of time steps of all agents.
        float makeSpan;     ///< Shows the maximum value of time steps of amoung all agents.
        int   collisions;   ///< Shows the number of collisions between agents while execution of task.
        int   collisionsObst;///< Shows the number of collisions between agents and static obstacles while execution of task.

};


#endif //ORCA_SUMMARY_H
