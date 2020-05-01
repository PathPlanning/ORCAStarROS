#include "Map.h"
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

#ifndef ORCA_SUBMAP_H
#define ORCA_SUBMAP_H




class SubMap
{
    public:
        SubMap();
        SubMap(const Map *map, Node mapOrigin, std::pair<int, int> mapSize, int divK);
        SubMap(const SubMap &obj);
        ~SubMap();

        bool CellIsObstacle(int i, int j) const;
        bool CellOnGrid(int i, int j) const;
        bool CellIsTraversable(int i, int j) const;
        bool CellIsTraversable(int i, int j, const std::unordered_set<Node> &occupiedNodes) const;
        int GetHeight() const;
        int GetWidth() const;
        int GetEmptyCellCount() const;
        int GetCellDegree(int i, int j) const;
        float GetCellSize() const;

        Node GetClosestNode(const Point &point) const;
        Point GetPoint(const Node &node) const;
        Node FindAvailableNode(Node start, std::unordered_map<int, Node> occupied);

        SubMap & operator = (const SubMap &obj);

    private:
        float cellSize;
        Node origin;
        std::pair<int, int> size;
        int divKoef;
        const Map *fullMap;
        int emptyCells;


};


#endif //ORCA_SUBMAP_H
