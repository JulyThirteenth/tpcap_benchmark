#pragma once

#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <algorithm>

namespace TPCAP
{
    template <typename T>
    struct TemplatePoint
    {
        T x, y;
        TemplatePoint(T x_, T y_) : x(x_), y(y_) {}
        TemplatePoint() : x(0), y(0) {}
    };
    template <typename T>
    bool operator==(const struct TemplatePoint<T> &l, const struct TemplatePoint<T> &r)
    {
        return (l.x == r.x) && (l.y == r.y);
    }
    typedef struct TemplatePoint<double> Point;
    typedef struct TemplatePoint<int> intPoint;

    template <typename T1, typename T2>
    struct TemplateVector
    {
        T1 x, y;
        T2 t;
        TemplateVector(T1 x_, T1 y_, T2 t_) : x(x_), y(y_), t(t_) {}
        TemplateVector() : x(0), y(0), t(0) {}
        struct TemplateVector<T1, T2> &operator=(const struct TemplateVector<T1, T2> &vec)
        {
            if (this != &vec)
            {
                this->x = vec.x;
                this->y = vec.y;
                this->t = vec.t;
            }
            return *this;
        }
    };
    typedef struct TemplateVector<double, double> Vector;

    class GridMap
    {
    public:
        int cols;
        int rows;
        double resolution;
        std::vector<std::vector<bool>> occs;
    };


    typedef struct edge
    {
        int ymax;
        double x;
        double dx;
        struct edge *next;
    } Edge;
    typedef Edge **EdgeTable;
    typedef Edge *ActiveEdgeTable;

    void fillPolygon(const std::vector<Point> &vertexes, double resolution, std::vector<intPoint> &polygArea);
}

#endif
