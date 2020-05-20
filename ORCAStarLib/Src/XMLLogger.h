/*!
\file
\brief File contains XMLLogger class.
*/


#ifndef ORCA_XMLLOGGER_H
#define ORCA_XMLLOGGER_H

#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>

#include "../tinyxml/tinyxml2.h"
#include "Logger.h"

using namespace std;
using namespace tinyxml2;


/*!
 * \brief XMLLogger class implements interface for system state logging to XML-file.
 * \details Log contains the full information about the execution of task.
 *          It includes same tags as input file, summary and information about each step of each agent.
 *          Summary contains the main information about the execution of tasks.
 *          Summary includes following tags:
 *
 *          * successrate — shows the percent of agents, which succeed their tasks;
 *          * runtime — shows the time of running of task;
 *          * flowtime — shows the sum of steps of all agents;
 *          * makespan — shows the maximum value of steps of amoung all agents;
 *          * collisions — shows the number of collisions between agents while execution of task;
 *          * collisionsobst — shows the number of collisions between agents and static obstacles while execution of task;
 *
 *          Summary example:
 *          \code{.xml}
 *           <summary successrate="100" runtime="1.061" makespan="170.90001" flowtime="477.60001" collisions="0" collisionsobst="0"/>
 *          \endcode
 *
 *          Agent's path example:
 *          \code{.xml}
 *          <agent number="0">
 *              <path pathfound="true" steps="4">
 *                 <step number="0" x="33.18066" y="9.1728058"/>
 *                 <step number="1" x="33.36132" y="9.3456116"/>
 *                 <step number="2" x="33.541981" y="9.5184174"/>
 *                 <step number="3" x="33.722641" y="9.6912231"/>
 *             </path>
 *          </agent>
 * \endcode
 *
 * \ingroup ORCAStarLib
 */

class XMLLogger : public Logger
{
    public:
        /*!
         * \brief XMLLogger default constructor.
         */
        XMLLogger();

        /*!
         * \brief XMLLogger constructor.
         * \param fileName Path to output log file.
         * \param inpFileName Path to input file to copy to output.
         */
        XMLLogger(std::string fileName, std::string inpFileName);

        /*!
         * \brief XMLLogger copy constructor.
         * \param obj Object to copy.
         */
        XMLLogger(const XMLLogger &obj);

        /*!
         * \brief XMLLogger destructor.
         */
        ~XMLLogger() override;

        /*!
         * \brief Method for creating final log in XML-file.
         * \return Succsess of log creating.
         */
        bool GenerateLog() override;

        /*!
         * \brief Sets brief information about execution results.
         * \param res Execution results in brief form.
         */
        void SetSummary(const Summary &res) override;

        /*!
         * \brief Sets full information about execution, including
         *        states on each step, curren goals on each step, success of each agent
         * \param stepsLog Agent states on each step.
         * \param goalsLog Agent curren goals on each step.
         * \param resultsLog Success of each agent and number of steps of each agent.
         */
        void
        SetResults(const std::unordered_map<int, std::vector<Point>> &stepsLog, const std::unordered_map<int, std::vector<Point>> &goalsLog,
                   const std::unordered_map<int, std::pair<bool, int>> &resultsLog) override;

        /*!
         * \brief Method for cloning objects.
         * Implementations of this method creates copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        XMLLogger* Clone() const override;

        /*!
         * \brief Creates a string of form "inpFileName_agentsNum_log.xml"
         * \param inpFileName Path to input XML-file to insert into result string.
         * \param agentsNum Number of agents to insert into result string.
         * \return String of form "inpFileName_agentsNum_log.xml"
         */
        static std::string GenerateLogFileName(std::string inpFileName, int agentsNum);

    private:
        //! \cond
        bool CloneInputFile();

        std::string fileName;
        std::string inpFileName;
        XMLDocument *doc;
        XMLElement *root;
        XMLElement *log;
        //! \endcond
};


#endif //ORCA_XMLLOGGER_H
