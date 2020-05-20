/*!
\file
\brief File contains Node, Point, Line, Vertex, ObstacleSegment classes and some methods and functions implementations.
*/

#include <cmath>
#include <vector>
#include <string>
#include <cstddef>
#include <tuple>

#include "Const.h"

#ifndef ORCA_GEOM_H
#define ORCA_GEOM_H

/*!
 * \brief The Node class defines a cell of grid (see \ref Map "Map class")
 * \ingroup ORCAStarLib
 */
class Node
{
    public:
        int     i; ///< The number of row. Part of (i,j) address of cell on grid.
        int     j; ///< The number of column. Part of (i,j) address of cell on grid.
        double  g; ///< g-value of node for A* based algorithms.
        double  H; ///< h-value of node for A* based algorithms.
        double  F;  ///< F = g + h. Value for A* based algorithms.
        Node    *parent; ///< The node from which the transition to this node was made. Value for A* based algorithms.

        /*!
         * \brief Node constructor with parameters.
         * \param i The number of row. Part of (i,j) address of cell on grid.
         * \param j The number of column. Part of (i,j) address of cell on grid.
         * \param p The node from which the transition to this node was made. Optional. Value for A* based algorithms.
         * \param g g-value of node for A* based algorithms. Optional.
         * \param h h-value of node for A* based algorithms. Optional.
         */
        Node(int i = 0, int j = 0, Node *p = nullptr, double g = 0, double h = 0)
            : i(i), j(j), g(g), H(h), F(g+h), parent(p){}

        /*!
         * \brief operator !=
         * \param other Object to compare
         * \return Inequality of nodes addresses
         */
        bool operator != (const Node &other) const ;

        /*!
         * \brief operator ==
         * \param another Object to compare
         * \return Equality of nodes addresses
         */
        bool operator == (const Node &another) const;
};

/*!
 * \brief The Point class defines a position (or euclidean vector from (0,0)) in 2D space.
 * \ingroup ORCAStarLib
 */
class Point
{
    public:
        /*!
         * \brief Point default constructor.
         */
        Point();

        /*!
         * \brief Point constructor with parameters.
         * \param x X-coordinate of the point.
         * \param y Y-coordinate of the point.
         */
        Point(float x, float y);

        /*!
         * \brief Point copy constructor.
         * \param obj An object to copy.
         */
        Point(const Point &obj);

        /*!
         * \brief Default destructor.
         */
        virtual ~Point() = default;

        /*!
         * \brief Returns X-coordinate of the point.
         * \return X-coordinate of the point.
         */
        float X() const;

        /*!
         * \brief Returns Y-coordinate of the point.
         * \return Y-coordinate of the point.
         */
        float Y() const;

        /*!
         * \brief Creates STL pair (x,y) from point.
         * \return STL pair (x,y).
         */
        std::pair<float, float> GetPair();

        /*!
         * \brief Computes scalar product of vectors (this * anoher)
         * \param another Second operand of scalar product
         * \return Result of scalar product of vectors (this * anoher)
         */
        float ScalarProduct(const Point &another) const;

        /*!
         * \brief Computes euclidean norm of vector.
         * \return Euclidean norm of vector.
         */
        float EuclideanNorm() const;

        /*!
         * \brief Computes squared euclidean norm of vector.
         * \return Squared euclidean norm of vector.
         */
        float SquaredEuclideanNorm() const;

        /*!
         * \brief Computes the determinant of matrix.
         * \details
         * <p align="center">
         * <img width="300" src="./images/det.jpeg" alt="Map sheme" >
         * </p>
         * \param another Second column of matrix
         * \return The determinant of matrix.
         */
        float Det(Point another) const;

        /*!
         * \brief Creates STL string, which contains x,y values.
         * \return STL string, which contains x,y values.
         */
        std::string ToString() const;

        /*!
         * \brief operator -
         * \param another Second operand
         * \return (this.x - another.x, this.y - another.y)
         */
        Point operator - (const Point &another) const;

        /*!
         * \brief operator +
         * \param another Second operand
         * \return (this.x + another.x, this.y + another.y)
         */
        Point operator + (const Point &another) const;

        /*!
         * \brief operator ==
         * \param another Object to compare.
         * \return (x, y) == (another.x, another.y)
         */
        bool operator == (const Point &another) const;

        /*!
         * \brief operator *
         * \param k Scalar multiplier.
         * \return (k * x, k * y)
         */
        Point operator * (float k) const;

        /*!
         * \brief operator /
         * \param k Scalar divider.
         * \return ( x / k, y / k)
         */
        Point operator / (float k) const;

        /*!
         * \brief operator -
         * \return (-x, -y)
         */
        Point operator - () const;

        /*!
         * \brief Assignment operator
         * \param obj Object to assign
         * \return Reference to object.
         */
        virtual Point & operator = (const Point &obj);



    private:
        float x; ///< X-coordinate of the point.
        float y; ///< Y-coordinate of the point.
};


/*!
 * \brief The class defines a line in 2D space.
 * \details The line is determined by a point and a direction vector.
 * \ingroup ORCAStarLib
 */
class Line
{
    public:
        Vector dir; ///< direction vector of line.
        Point liesOn; ///< point on line.
};

/*!
 * \brief The Vertex class defines a vertex of an obstacle polygon.
 * \ingroup ORCAStarLib
 */
class Vertex : public Point
{
    public:
        /*!
         * \brief Vertex default constructor.
         */
        Vertex() : Point() {}

        /*!
         * \brief Vertex constructor.
         * \param x X-coordinate of the vertex.
         * \param y Y-coordinate of the vertex.
         * \param cvx Convexity of vertex
         */
        Vertex(float x, float y, bool cvx = false) : Point(x, y), convex(cvx) {}

        /*!
         * \brief Vertex constructor
         * \param obj Position of vertex
         * \param cvx Convexity of vertex
         */
        Vertex(const Point &obj, bool cvx = false) : Point(obj), convex(cvx) {}

        /*!
         * \brief Vertex copy constructor.
         * \param obj Object to copy.
         */
        Vertex(const Vertex &obj) : Point(obj), convex(obj.convex) {}

        /*!
         * \brief Default destructor.
         */
        ~Vertex() override = default;

        /*!
         * \brief Return convexity of vertex.
         * \return Convexity of vertex.
         */
        bool IsConvex() const;

        /*!
         * \brief Sets convexity of vertex.
         * \param cvx Convexity of vertex.
         */
        void SetConvex(bool cvx);

        /*!
         * \brief Assignment operator
         * \param obj Object to assign
         * \return Reference to object.
         */
        Vertex & operator = (const Vertex &obj);

    private:
        bool convex; ///< Convexity of vertex.
};

/*!
 * \brief The ObstacleSegment class defines an edge of an obstacle polygon.
 * \ingroup ORCAStarLib
 */
class ObstacleSegment
{
    public:
     /*!
      * \brief Default constructor.
      */
        ObstacleSegment() = default;

        /*!
         * \brief Copy constructor.
         * \param obj Object to copy.
         */
        ObstacleSegment(const ObstacleSegment &obj) : left(obj.left), right(obj.right), id(obj.id), next(obj.next), prev(obj.prev), dir(obj.dir) {}

        /*!
         * \brief ObstacleSegment constructor.
         * \param id Identifier of obstacle edge.
         * \param left Left vertex of edge (when viewed from the outside of an obstacle).
         * \param right Right vertex of edge (when viewed from the outside of an obstacle).
         */

        ObstacleSegment(int id, const Vertex &left, const Vertex &right) : left(left), right(right), id(id) {dir = right-left; dir = dir/dir.EuclideanNorm();}

        /*!
         * \brief Default destructor.
         */
        ~ObstacleSegment() = default;

        /*!
         * \brief operator ==
         * \param another Object to comapare
         * \return (id == another.id)
         */
        bool operator == (const ObstacleSegment &another) const;

        /*!
         * \brief Assignment operator
         * \param obj Object to assign
         * \return Reference to object.
         */
        ObstacleSegment & operator = (const ObstacleSegment &obj);


        int id; ///< Identifier of edge
        Vertex left; ///< Left vertex of edge (when viewed from the outside of an obstacle).
        Vertex right; ///< Right vertex of edge (when viewed from the outside of an obstacle).
        Vector dir; ///< Vector (right-left)/|right-left|
        ObstacleSegment *next; ///< Next edge in obstacle.
        ObstacleSegment *prev; ///< Previous edge in obstacle.

};

/*!
 * \brief The set of utility functions.
 */
namespace Utils
{
/*!
     * \brief       Computes squared euclidean distance from point to line segment.
     * \param L1    First point of line segment.
     * \param L2    Last point of line segment
     * \param P     Point,for which the distance is computed
     * \return      Squared euclidean distance from point P to line segment (L1-L2).
     * \author      https://github.com/snape/RVO2
     */
    float SqPointSegDistance(Point L1, Point L2, Point P);

    /*!
     * \brief      Solves a one-dimensional linear program on a specified line
     *             subject to linear constraints defined by lines and a circular
     *             constraint.
     * \param      lines         Lines defining the linear constraints.
     * \param      curr          The specified line constraint.
     * \param      radius        The radius of the circular constraint.
     * \param      optVelocity   The optimization velocity.
     * \param      directionOpt  True if the direction should be optimized.
     * \param      result        A reference to the result of the linear program.
     * \return     True if successful.
     * \author    https://github.com/snape/RVO2
     */
    bool linearProgram1(const std::vector<Line> &lines, unsigned long curr, float radius, const Vector &optVelocity,
                               bool directionOpt, Vector &result);

    /*!
     * \brief      Solves a two-dimensional linear program subject to linear
     *             constraints defined by lines and a circular constraint.
     * \param      lines         Lines defining the linear constraints.
     * \param      radius        The radius of the circular constraint.
     * \param      optVelocity   The optimization velocity.
     * \param      directionOpt  True if the direction should be optimized.
     * \param      result        A reference to the result of the linear program.
     * \return     The number of the line it fails on, and the number of lines if successful.
     * \author    https://github.com/snape/RVO2
     */
    unsigned long int linearProgram2(const std::vector<Line> &lines, float radius, const Vector &optVelocity,
                                bool directionOpt, Vector &result);

    /*!
     * \brief      Solves a two-dimensional linear program subject to linear
     *             constraints defined by lines and a circular constraint.
     * \param      lines         Lines defining the linear constraints.
     * \param      numObstLines  Count of obstacle lines.
     * \param      beginLine     The line on which the 2-d linear program failed.
     * \param      radius        The radius of the circular constraint.
     * \param      result        A reference to the result of the linear program.
     * \author    https://github.com/snape/RVO2
     */
    void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
                                float radius, Vector &result);


    template<typename T>
    /*!
     * \brief Comparison function for pairs of the form <float, T> For sorting purposes.
     * \param a First operand.
     * \param b Second operand
     * \return Is a more than b
     */
    bool More( std::pair<float, T> a, std::pair<float, T> b)
    {
        return (a.first > b.first);
    }

    template<typename T>
    /*!
     * \brief Comparison function for pairs of the form <float, T> For sorting purposes.
     * \param a First operand.
     * \param b Second operand
     * \return Is a less than b
     */
    bool Less( std::pair<float, T> a, std::pair<float, T> b)
    {
        return (a.first < b.first);
    }
};





/*********************************************************
 *                Methods Implementations                *
 *********************************************************/


inline bool Node::operator == (const Node &another) const
{
    return i == another.i && j == another.j;
}

inline bool Node::operator != (const Node &other) const
{
    return i != other.i || j != other.j;
}




inline bool ObstacleSegment::operator ==(const ObstacleSegment &another) const
{
    return (this->id == another.id);
}

inline ObstacleSegment &ObstacleSegment::operator = (const ObstacleSegment &obj)
{

    id = obj.id;
    left = obj.left;
    right = obj.right;
    dir = obj.dir;
    next = obj.next;
    prev = obj.prev;
    return *this;
}

inline float Point::ScalarProduct(const Point &another) const
{
    return this->x * another.x + this->y * another.y;
}


inline Point Point::operator - (const Point &another) const
{
    return {this->x - another.x, this->y - another.y};
}


inline Point Point::operator + (const Point &another) const
{
    return {this->x + another.x, this->y + another.y};
}


inline Point Point::operator * (float k) const
{
    return {this->x * k, this->y * k};
}


inline Point Point::operator /(float k) const
{
    const float invK = 1.0f / k;
    return {this->x * invK, this->y * invK};
}


inline float Point::SquaredEuclideanNorm() const
{
    return this->ScalarProduct(*this);
}


inline float Point::EuclideanNorm() const
{
    return std::sqrt(this->ScalarProduct(*this));
}


inline float Point::Det(Point another) const
{
    return (this->x * another.y - this->y * another.x);
}


inline bool Point::operator ==(const Point &another) const
{
    return (this->x == another.x) && (this->y == another.y);
}


inline Point Point::operator-() const
{
    return Point(-this->x, -this->y);
}


inline Point& Point::operator = (const Point &obj)
{
    x = obj.x;
    y = obj.y;
    return *this;
}


inline Vertex& Vertex::operator = (const Vertex &obj)
{
    Point::operator=(obj);
    convex = obj.convex;
    return *this;
}



#endif //ORCA_GEOM_H
