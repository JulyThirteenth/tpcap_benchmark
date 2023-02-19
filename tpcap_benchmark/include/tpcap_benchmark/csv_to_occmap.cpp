#include "csv_to_occmap.h"

namespace TPCAP
{

    CsvToOccmap::CsvToOccmap()
    {
        pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        pub_shift = nh.advertise<geometry_msgs::PointStamped>("/corrdShift", 1, true);
    }

    void CsvToOccmap::pubMapAndShift()
    {
        double resolution;
        nh.param("/tpcap_benchmark_node/case_resolution", resolution, 0.1);
        CsvToGridMap ctg(resolution);
        GridMap grid_map;
        if (ctg.getGridMap(grid_map))
        {
            geometry_msgs::PointStamped corrd_shift;
            corrd_shift.point.x = ctg.map_info.corrd_shift.x;
            corrd_shift.point.y = ctg.map_info.corrd_shift.y;
            corrd_shift.header.frame_id = "map";
            corrd_shift.header.stamp = ros::Time::now();
            pub_shift.publish(corrd_shift);
            nav_msgs::OccupancyGrid occ_map;
            occ_map.info.width = ctg.map_info.width;
            occ_map.info.height = ctg.map_info.heigth;
            occ_map.info.resolution = ctg.map_info.resolution;
            occ_map.data.resize(ctg.map_info.width * ctg.map_info.heigth);
            for (int i = 0; i < grid_map.cols; i++)
            {
                for (int j = 0; j < grid_map.rows; j++)
                {
                    occ_map.data[i + j * ctg.map_info.width] = (grid_map.occs[i][j] == false ? 0 : 100);
                }
            }
            occ_map.header.frame_id = "map";
            occ_map.header.stamp = ros::Time::now();
            pub_map.publish(occ_map);
        }
        else
        {
            ROS_ERROR("parseCsv falied!");
        }
    }
}