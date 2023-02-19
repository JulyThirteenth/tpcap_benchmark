#ifndef _CSV_TO_OCCMAP_H_
#define _CSV_TO_OCCMAP_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include "csv_to_gridmap.h"

namespace TPCAP
{
    //作用：实现 csv 数据到占用栅格地图的转换
    class CsvToOccmap
    {
    public:
        CsvToOccmap();
        //根据地图信息发布占用栅格地图和偏移坐标
        void pubMapAndShift();
    private:
        GridMap grid_map;
        ros::NodeHandle nh;
        ros::Publisher pub_map;
        //对每一个 Case 中的地图信息做了偏移处理，故发布出偏移信息：x 轴，y 轴上的偏移量
        ros::Publisher pub_shift; 
    };
}

#endif