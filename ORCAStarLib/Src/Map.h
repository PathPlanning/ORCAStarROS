/*!
\file
\brief File contains Map class.
*/


#include <string>
#include <vector>

#include "Geom.h"


#ifndef ORCA_MAP_H
#define ORCA_MAP_H


/*!
 * \brief Map class describes static environment.
 * \details This class contains main information about static environment.
 * The map is set in the form of a grid, are also described in the form of polygons.
 * The grid is a rectangular matrix of square cells, each cell can be traversable or not.
 * The size of cell side is set by the value of cellSize.
 * The address of each cell on the grid is specified by a pair (i, j),
 * where i is the row number of the matrix, j is the column number of the matrix
 * The environment description is specified in the orthogonal coordinates.
 * Every obstacle in form of polygon is vector of points in orthogonal coordinates.
 * Vertices of the simple obstacle shold be listed in counterclockwise order, vertices of boundary in clockwise order.
 * An illustration of the environment is presented in the figure below.
 *
 * <p align="center">
 * <img width="500" src="./images/map.png" alt="Map sheme" >
 * </p>
 *
 * \ingroup ORCAStarLib
 */
class Map
{
    public:
        /*!
         * \brief Map default constructor.
         */
        Map();

        /*!
         * \brief Map constructor with parameters.
         * \param cellSize Size of cell side.
         * \param grid Matrix of traversability of environment. Every cell can be 0 (traversable) or 1 (obstacle).
         * \param obstacles Vector of obstacles in form of polygons. Every obstacle is vector of points in orthogonal coordinates.
         * Vertices of the simple obstacle shold be listed in counterclockwise order, vertices of boundary in clockwise order.
         */
        Map(float cellSize, std::vector<std::vector<int>> &grid, std::vector<std::vector<Point>> &obstacles);

        /*!
         * \brief Map copy constructor.
         * \param obj Object to copy.
         */
        Map(const Map &obj);

        /*!
         * \brief Map destructor.
         */
        ~Map();

        /*!
         * \brief Check Cell (i,j) is obstacle
         * \param i Row number in the matrix.
         * \param j Column number in the matrix
         * \return True if obstacle, otherwise false.
         */
        bool CellIsObstacle(int i, int j) const;

        /*!
         * \brief Check Cell (i,j) is inside grid.
         * \param i Row number in the matrix.
         * \param j Column number in the matrix
         * \return True if inside grid, otherwise false.
         */
        bool CellOnGrid(int i, int j) const;

        /*!
         * \brief Check Cell (i,j) is traversable
         * \param i Row number in the matrix.
         * \param j Column number in the matrix
         * \return False if obstacle, otherwise true.
         */
        bool CellIsTraversable(int i, int j) const;

        /*!
         * \brief Return height of grid.
         * \return Number of rows in grid matrix
         */
        unsigned int GetHeight() const;

        /*!
         * \brief Return width of grid.
         * \return Number of columns in grid matrix
         */
        unsigned int GetWidth() const;

        /*!
         * \brief Return size of cell side.
         * \return Size of cell side.
         */
        float GetCellSize() const;

        /*!
         * \brief Return obstacles in form of polygons.
         * \return Vector of polygons.
         */
        const std::vector<std::vector<ObstacleSegment>>& GetObstacles() const;

        /*!
         * \brief Return cell to which point in orthogonal coordinates belong.
         * \param point Point in orthogonal coordinates .
         * \return Cell of grid matrix.
         */
        Node GetClosestNode(const Point &point) const;

        /*!
         * \brief Return center of grid cell in orthogonal coordinates.
         * \param node Grid cell.
         * \return Position of grid cell center in orthogonal coordinates.
         */
        Point GetPoint(const Node &node) const;

        /*!
         * \brief Assignment operator.
         * \param obj Map to assignment.
         * \return Reference to map.
         */
        Map & operator = (const Map &obj);

    private:
        //!\cond
        float cellSize;
        unsigned int height;
        unsigned int width;
        std::vector<std::vector<int>> *grid;
        std::vector<std::vector<ObstacleSegment>> *obstacles;
        //!\endcond
};


#endif //ORCA_MAP_H
