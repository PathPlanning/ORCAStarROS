/*!
\file
\brief File contains LineOfSight class.
*/

#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H


#define CN_OBSTACLE 1

#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>

#include "Const.h"


/*!
 * \brief This class implements line-of-sight function between two cells on the grid for a variable size of agent.
 * \details It also has a method for checking cell's traversability.
 * For its work is needed the size of agent and a map container that has 'cellIsObstacle' and 'cellOnGrid' methods.
 * \author Andreychuk A.
 * \date 2017
 * \ingroup ORCAStarLib
 */


class LineOfSight
{
public:
    /*!
     * \brief LineOfSight constructor.
     * \param agentSize Size of Agent relative to grid cell size.
     */
    LineOfSight(double agentSize = 0.5)
    {
        this->agentSize = agentSize;
        int add_x, add_y, num = agentSize + 0.5 - CN_EPS;
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
            {
                add_x = x != 0 ? 1 : 0;
                add_y = y != 0 ? 1 : 0;
                if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                    cells.push_back({x, y});
            }
        if(cells.empty())
            cells.push_back({0,0});
    }

    /*!
     * \brief Set size of agent.
     * \param agentSize Size of Agent relative to grid cell size.
     */
    void setSize(double agentSize)
    {
        this->agentSize = agentSize;
        int add_x, add_y, num = agentSize + 0.5 - CN_EPS;
        cells.clear();
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
            {
                add_x = x != 0 ? 1 : 0;
                add_y = y != 0 ? 1 : 0;
                if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                    cells.push_back({x, y});
            }
        if(cells.empty())
            cells.push_back({0,0});
    }

    /*!
     * \brief Returns all cells on grid that are affected by agent during moving along a line.
     * \param x1 i-coordinate of first cell of line.
     * \param y1 j-coordinate of first cell of line.
     * \param x2 i-coordinate of last cell of line.
     * \param y2 j-coordinate of last cell of line.
     * \param map Map object.
     * \return All cells that are affected by agent during moving along a line.
     */
    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByLine(int x1, int y1, int x2, int y2, const T &map)
    {
        std::vector<std::pair<int, int>> lineCells(0);
        if(x1 == x2 && y1 == y2)
        {
            for(auto cell:cells)
                lineCells.push_back({x1+cell.first, y1+cell.second});
            return lineCells;
        }
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x >= delta_y && x1 > x2) || (delta_y > delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int k, num;
        std::pair<int, int> add;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPS;

        if(delta_x >= delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPS;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 - n*step_x, y1 + k*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 + n*step_x, y2 - k*step_y});
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x+=step_x)
            {
                lineCells.push_back({x, y});
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y + k*step_y});
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y - k*step_y});
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPS;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 + k*step_x, y1 - n*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 - k*step_x, y2 + n*step_y});
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                lineCells.push_back({x, y});
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x + k*step_x, y});
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x - k*step_x, y});
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        for(k = 0; k < cells.size(); k++)
        {
            add = {x1 + cells[k].first, y1 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
            add = {x2 + cells[k].first, y2 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
        }

        for(auto it = lineCells.begin(); it != lineCells.end(); it++)
            if(!map.CellOnGrid(it->first, it->second))
            {
                lineCells.erase(it);
                it = lineCells.begin();
            }
        return lineCells;
    }

    /*!
     * \brief Checks traversability of all cells on grid affected by agent's body
     * \param x i-coordinate of cell.
     * \param y j-coordinate of cell.
     * \param map Map object.
     * \return Traversability of all cells affected by agent's body
     */
    template <class T>
    bool checkTraversability(int x, int y, const T &map)
    {
        for(int k = 0; k < cells.size(); k++)
            if(!map.CellOnGrid(x + cells[k].first, y + cells[k].second) || map.CellIsObstacle(x + cells[k].first, y + cells[k].second))
                return false;
        return true;
    }


    /*!
     * \brief Checks line-of-sight between two cells.
     * \param x1 i-coordinate of first cell of line.
     * \param y1 j-coordinate of first cell of line.
     * \param x2 i-coordinate of last cell of line.
     * \param y2 j-coordinate of last cell of line.
     * \param map Map object.
     * \return True, if there is a line of sight between the cells, otherwise - false.
     */
    template <class T>
    bool checkLine(int x1, int y1, int x2, int y2, const T &map)
    {
        //std::cout<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<" "<<"\n";
        if(!checkTraversability(x1, y1, map) || !checkTraversability(x2, y2, map)) //additional cheсk of start and goal traversability,
            return false;                                                //it can be removed if they are already checked

        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPS;
        int k, num;

        if(delta_x > delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPS;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x1 - n*step_x, y1 + k*step_y))
                        if(map.CellIsObstacle(x1 - n*step_x, y1 + k*step_y))
                            return false;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x2 + n*step_x, y2 - k*step_y))
                        if(map.CellIsObstacle(x2 + n*step_x, y2 - k*step_y))
                            return false;
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x+=step_x)
            {
                if(map.CellIsObstacle(x, y))
                    return false;
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x, y + k*step_y))
                            return false;
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x, y - k*step_y))
                            return false;
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPS;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x1 + k*step_x, y1 - n*step_y))
                        if(map.CellIsObstacle(x1 + k*step_x, y1 - n*step_y))
                            return false;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x2 - k*step_x, y2 + n*step_y))
                        if(map.CellIsObstacle(x2 - k*step_x, y2 + n*step_y))
                            return false;
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                if(map.CellIsObstacle(x, y))
                    return false;
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x + k*step_x, y))
                            return false;
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x - k*step_x, y))
                            return false;
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        return true;
    }


    /*!
     * \brief Returns all cells on grid that are affected by agent in position (i,j).
     * \param i i-coordinate of first cell of line.
     * \param j j-coordinate of first cell of line.
     * \return All cells that are affected by agent in position (i,j).
     */
    std::vector<std::pair<int, int>> getCells(int i, int j)
    {
        std::vector<std::pair<int, int>> cells;
        for(int k=0; k<this->cells.size(); k++)
            cells.push_back({i+this->cells[k].first,j+this->cells[k].second});
        return cells;
    }
private:
    //! \cond
    double agentSize;
    std::vector<std::pair<int, int>> cells; //cells that are affected by agent's body
    //!\endcond
};

#endif // LINEOFSIGHT_H
