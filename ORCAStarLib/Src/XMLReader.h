/*!
\file
\brief File contains XMLReader class.
*/


#include <string>
#include <sstream>

#include "../tinyxml/tinyxml2.h"
#include "Reader.h"
#include "Const.h"
#include "Agent.h"
#import "ORCAAgent.h"
#include "ThetaStar.h"
#include "DirectPlanner.h"



#ifndef ORCA_XMLREADER_H
#define ORCA_XMLREADER_H



using namespace tinyxml2;

/*!
 * \brief The XMLReader class implements interface for input data reading from XML-file with a specific structure.
 *
 * Input file should contain:
 *  *  Mandatory tag `<agents>`. It describes the parameters of the agents.
 *      *  `number` — mandatory attribute that define the number of agents;
 *      *  `<default_parameters>` — mandatory tags that defines default parameters of agents and agent's perception.
 *          *  `agentsmaxnum` — mandatory attribute that defines a number of neighbors, that the agent takes into account;
 *          *  `movespeed` — mandatory attribute that defines maximum speed of agent;
 *          *  `sightradius` — mandatory attribute that defines the radius in which the agent takes neighbors into account;
 *          *  `size` — mandatory attribute that defines size of the agent (radius of the agent);
 *          *  `timeboundary` — mandatory attribute that defines the time within which the algorithm ensures collision avoidance with other agents;
 *          *  `timeboundaryobst` — mandatory attribute that defines the time within which the algorithm ensures collision avoidance with static obstacles.
 *  *  `<agent>` — mandatory tags that defines parameters of each agent.
 *      *  `id` — mandatory attribute that defines the identifier of agent;
 *      *  `start.xr` — mandatory attribute that defines the coordinate of start position on the x-axis (hereinafter, excluding `map` tag, points (x,y) are in coordinate system, which has an origin (0,0) in lower left corner. More about coordinate systems in the illustration below);
 *      *  `start.yr` — mandatory attribute that defines the coordinate of start position on the y-axis;
 *      *  `goal.xr` — mandatory attribute that defines the coordinate of finish position on the x-axis;
 *      *  `goal.yr` — mandatory attribute that defines the coordinate of finish position on the y-axis;
 *      *  `agentsmaxnum` — attribute that defines a number of neighbors, that the agent takes into account;
 *      *  `movespeed` — attribute that defines maximum speed of agent;
 *      *  `sightradius` — attribute that defines the radius in which the agent takes neighbors into account;
 *      *  `size` — attribute that defines size of the agent (radius of the agent);
 *      *  `timeboundary` — attribute that defines the time within which the algorithm ensures collision avoidance with other agents;
 *      *  `timeboundaryobst` — attribute that define the time within which the algorithm ensures collision avoidance with static obstacles.
 *  *  Mandatory tag `<map>`. It describes the environment for global path planning.
 *      *  `<height>` and `<width>` — mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, ( * * width * * -1,  * * height * * -1) is lower right (more about coordinate systems in the illustration below).
 *      *  `<cellsize>` — optional tag that defines the size of one cell.
 *      *  `<grid>` — mandatory tag that describes the square grid constituting the map. It consists of `<row>` tags. Each `<row>` contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" — for untraversable (actually any other figure but "0" can be used instead of "1").
 *  *  Mandatory tag `<obstacles>`. It describes static obstacles for collision avoidance.
 *      *  `number` — mandatory attribute that defines the number of obstacles;
 *      *  `<obstacle>` — mandatory tags which defines each static obstacles for collision avoidance.
 *      *  `<vertex>` — mandatory tags which defines vertex of static obstacle for collision avoidance.
 *          *   `xr` — mandatory attribute that defines the coordinate of vertex on the x-axis;
 *          *   `yr` — mandatory attribute that defines the coordinate of vertex on the y-axis.
 *  *  Mandatory tag `<algorithm>`. It describes the parameters of the algorithm.
 *      *  `<delta>` — mandatory tag that defines the distance between the center of the agent and the finish, which is enough to reach the finish (ORCA parameter);
 *      *  `<timestep>` — mandatory tag that defines the time step of simulation (ORCA parameter);
 *      *  `<searchtype>` — tag that defines the type of planning. Possible values - "thetastar" (use Theta* for planning), "direct" (turn off global planning and always use direction to global goal). Default value is "thetastar" (global planning parameter);
 *      *  `<breakingties>` — tag that defines the priority in OPEN list for nodes with equal f-values. Possible values - "0" (break ties in favor of the node with smaller g-value), "1" (break ties in favor of the node with greater g-value). Default value is "0" (Theta * *  parameter);
 *      *  `<cutcorners>` — boolean tag that defines the possibilty to make diagonal moves when one adjacent cell is untraversable. The tag is ignored if diagonal moves are not allowed. Default value is "false" (Theta * *  parameter);
 *      *  `<allowsqueeze>` — boolean tag that defines the possibility to make diagonal moves when both adjacent cells are untraversable. The tag is ignored if cutting corners is not allowed. Default value is "false" (Theta * *  parameter);
 *      *  `<hweight>` — defines the weight of the heuristic function. Should be real number greater or equal 1. Default value is "1" (Theta * *  parameter);
 * \ingroup ORCAStarLib
 */

class XMLReader : public Reader
{
    public:

        /*!
         * \brief XMLReader default constructor.
         */
        XMLReader();

        /*!
         * \brief XMLReader constructor
         * \param fileName Path to input XML-file
         */
        XMLReader(const std::string &fileName);

        /*!
         * \brief XMLReader copy constructor.
         * \param obj Object to copy.
         */
        XMLReader(const XMLReader &obj);

        /*!
         * \brief XMLReader destructor.
         */
        ~XMLReader() override;

        /*!
         * \brief Starts input data reading from XML-file.
         * \return Success of file reading.
         */
        bool ReadData() override;

        /*!
         * \brief Creates object with static environment data.
         * \param [out] map Static environment data
         * \return Success of object creating and transfer process.
         */
        bool GetMap(Map **map) override;

        /*!
         * \brief Creates object with algorithms and environment parameters.
         * \param [out] envOpt Algorithms and environment parameters.
         * \return Success of object creating and transfer process.
         */
        bool GetEnvironmentOptions(EnvironmentOptions **envOpt) override;

        /*!
         * \brief Creates object for all agents.
         * \param [out] agents Vector of all agents of system. Objects in vector contain full information about each agent.
         * \param [in] numThreshold Maximum number of agents in system.
         * \return Success of objects creating and transfer process.
         */
        bool GetAgents(std::vector<Agent *> &agents, const int &numThreshold) override;

        /*!
         * \brief Method for cloning objects.
         * Create copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        XMLReader* Clone() const override;

        /*!
         * \brief Assignment operator.
         * \param obj Object to assign.
         * \return Reference to assigned object.
         */
        XMLReader & operator = (const XMLReader & obj);



    private:

        //! \cond
        std::string fileName;

        XMLDocument *doc;
        XMLElement *root;

        std::vector<Agent *> *allAgents;
        Map *map;
        EnvironmentOptions *options;
        std::vector<std::vector<int>> *grid;
        std::vector<std::vector<Point>> *obstacles;
        int plannertype;

        bool ReadMap();
        bool ReadAgents();
        bool ReadAlgorithmOptions();
        //! \endcond
};


#endif //ORCA_XMLREADER_H
