/*!
\file
\brief File contains Logger class.
*/


#ifndef ORCA_LOGGER_H
#define ORCA_LOGGER_H

#include <string>
#include <vector>

#include "Summary.h"
#include "Geom.h"
#include "Agent.h"


/*!
 * \brief Logger class implements base interface for system state logging.
 * \ingroup ORCAStarLib
 */
class Logger
{
    public:
        /*!
         * \brief Logger virtual destructor.
         */
        virtual ~Logger() {};

        /*!
         * \brief Abstract method for creating final log in file or another format.
         * \return Succsess of log creating.
         */
        virtual bool GenerateLog() = 0;

        /*!
         * \brief Abstract method. Should sets brief information about execution results
         * \param res Execution results in brief form.
         */
        virtual void SetSummary(const Summary &res) = 0;

        /*!
         * \brief Abstract method. Should sets full information about execution, including
         *        states on each step, curren goals on each step, success of each agent
         * \param stepsLog Agent states on each step.
         * \param goalsLog Agent curren goals on each step.
         * \param resultsLog Success of each agent and number of steps of each agent.
         */
        virtual void
        SetResults(const std::unordered_map<int, std::vector<Point>> &stepsLog, const std::unordered_map<int, std::vector<Point>> &goalsLog,
                   const std::unordered_map<int, std::pair<bool, int>> &resultsLog) = 0;

        /*!
         * \brief Abstract method for cloning inheritors objects.
         * Implementations of this method should create copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        virtual Logger* Clone() const = 0;
};


#endif //ORCA_LOGGER_H
