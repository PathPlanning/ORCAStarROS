/*!
\file
\brief File contains ThetaStar class.
*/

#include <list>
#include <unordered_map>
#include <iostream>

#include "PathPlanner.h"
#include "Geom.h"
#include "LineOfSight.h"

#ifndef ORCA_THETASTAR_H
#define ORCA_THETASTAR_H

/*!
 * \brief ThetaStar class implements Theta* algorithm, which creates path from start to goal position.
 *
 * \ingroup ORCAStarLib
 */
class ThetaStar : public PathPlanner
{
    public:

        /*!
         * \brief ThetaStar constructor with parametes.
         * \param map Static environment map. It contains information about grid and static obstacles boundaries. More about it see in \ref index "Main page" and \ref Map "Map class".
         * \param options Environment and algorithm options. See \ref EnvironmentOptions "EnvironmentOptions class".
         * \param start Start position of agent.
         * \param goal Goal position of agent.
         * \param radius Radius of agent.
         */
        ThetaStar(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal, const float &radius);

        /*!
         * \brief ThetaStar copy constructor.
         * \param obj Object to copy.
         */
        ThetaStar(const ThetaStar &obj);

        /*!
         * \brief ThetaStar destructor.
         */
        ~ThetaStar() override;

        /*!
         * \brief Returns current goal of agent from global path.
         * \param [in] curr Current position of agent.
         * \param [out] next Current goal of agent.
         */
        bool GetNext(const Point &curr, Point &next) override;

        /*!
         * \brief Finds path from global start to global goal position using Theta* algorithm.
         * \return Success of pathfinding.
         */
        bool CreateGlobalPath() override;

        /*!
         * \brief Method for cloning inheritors objects. Creates copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        ThetaStar* Clone() const override;

        /*!
         * \brief Assignment operator.
         * \param obj Object to assign.
         * \return Reference to assigned object.
         */
        ThetaStar & operator = (const ThetaStar &obj);

    private:

        //! \cond
        bool SearchPath(const Node &start, const Node &goal);
        bool StopCriterion() const;
        void AddOpen(Node newNode);
        Node FindMin();
        float ComputeHFromCellToCell(int i1, int j1, int i2, int j2) const;
        float Distance(int i1, int j1, int i2, int j2) const;
        Node ResetParent(Node current, Node parent);
        void MakePrimaryPath(Node curNode);
        std::list<Node> FindSuccessors(Node curNode);


        std::list<Point> currPath;

        std::unordered_map<int, Node>   close;
        std::vector<std::list<Node>>    open;
        int                             openSize;
        bool                            glPathCreated;
        LineOfSight                     visChecker;

        //! \endcond



};


#endif //ORCA_THETASTAR_H
