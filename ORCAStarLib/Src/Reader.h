/*!
\file
\brief File contains Reader class.
*/


#include <vector>

#include "Map.h"
#include "EnvironmentOptions.h"
#include "Agent.h"

#ifndef ORCA_READER_H
#define ORCA_READER_H


/*!
 * \brief Reader class implements base interface for input data reading.
 * \ingroup ORCAStarLib
 */
class Reader
{
    public:
        /*!
         * \brief Reader virtual destructor.
         */
        virtual ~Reader(){}

        /*!
         * \brief Abstract method. Should starts input data reading from file or from another source.
         * \return Success of data reading.
         */
        virtual bool ReadData() = 0;

        /*!
         * \brief Abstract method. Should creates object with static environment data.
         * \param [out] map Static environment data
         * \return Success of object creating and transfer process.
         */
        virtual bool GetMap(Map **map) = 0;

        /*!
         * \brief Abstract method. Should creates object with algorithms and environment parameters.
         * \param [out] envOpt Algorithms and environment parameters.
         * \return Success of object creating and transfer process.
         */
        virtual bool GetEnvironmentOptions(EnvironmentOptions **envOpt) = 0;

        /*!
         * \brief Abstract method. Should creates object for all agents.
         * \param [out] agents Vector of all agents of system. Objects in vector contain full information about each agent.
         * \param [in] numThreshold Maximum number of agents in system.
         * \return Success of objects creating and transfer process.
         */
        virtual bool GetAgents(std::vector<Agent *> &agents, const int &numThreshold) = 0;

        /*!
         * \brief Abstract method for cloning inheritors objects.
         * Implementations of this method should create copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        virtual Reader* Clone() const = 0;
};


#endif //ORCA_READER_H
