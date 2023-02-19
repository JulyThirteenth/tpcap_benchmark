#ifndef _CSV_TO_GRIDMAP_H_
#define _CSV_TO_GRIDMAP_H_

#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include "tpcap_utils.h"

namespace TPCAP
{
    typedef struct MapInfo
    {
        int width;
        int heigth;
        double resolution;
        std::vector<std::vector<Point>> obstacles;
        Point corrd_shift;
    } MapInfo;

    class CsvToGridMap
    {
    public:
        CsvToGridMap(double resolution);
        MapInfo map_info;
        bool getGridMap(GridMap &grid_map);
    private:
        bool parseCsv();
        ros::NodeHandle nh;
    };
}

#endif