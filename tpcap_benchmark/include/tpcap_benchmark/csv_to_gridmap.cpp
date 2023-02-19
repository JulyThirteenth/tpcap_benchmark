#include "csv_to_gridmap.h"

namespace TPCAP
{
    CsvToGridMap::CsvToGridMap(double resolution)
    {
        map_info.resolution = resolution;
    }

    bool CsvToGridMap::parseCsv()
    {
        std::cout << std::fixed << std::setprecision(6); //设置数据流保留小数点后6位
        int case_id;
        double padding;
        std::string stage;
        std::string cases_path;
        nh.param("/tpcap_benchmark_node/case_id", case_id, 0);
        nh.param("/tpcap_benchmark_node/case_padding", padding, 4.);
        nh.param<std::string>("/tpcap_benchmark_node/case_stage", stage, "stage");
        // ROS_INFO("%s", stage.c_str());
        nh.param<std::string>("/tpcap_benchmark_node/cases_path", cases_path, "cases_path");
        // ROS_INFO("%s", case_path.c_str());
        std::string case_path = (cases_path + stage + "/Case" + std::to_string(case_id) + ".csv");
        std::fstream case_reader;
        case_reader.open(case_path.c_str(), std::ios::in);
        if (!case_reader.is_open())
        {
            ROS_ERROR("%s", (std::string("Csv file is not exist: ") + case_path).c_str());
            return false;
        }
        std::string case_data;
        case_reader >> case_data;
        case_reader.close();
        Vector start, goal;
        int id = 0, sub_id = 0;
        for (int i = 0; i < 6; i++) //获取 csv 文件中起点与终点数据
        {
            while (case_data[id + sub_id] != ',')
            {
                sub_id++;
            }
            double temp = atof(case_data.substr(id, sub_id).c_str());
            switch (i)
            {
            case 0:
                start.x = temp;
                break;
            case 1:
                start.y = temp;
                break;
            case 2:
                start.t = temp;
                break;
            case 3:
                goal.x = temp;
                break;
            case 4:
                goal.y = temp;
                break;
            case 5:
                goal.t = temp;
            }
            id += (++sub_id);
            sub_id = 0;
        }
        while (case_data[id + sub_id] != ',') //障碍物个数
        {
            sub_id++;
        }
        int obstacle_num = atof(case_data.substr(id, sub_id).c_str());
        id += (++sub_id);
        sub_id = 0;
        std::vector<int> obstacle_vertex_num;
        for (int i = 0; i < obstacle_num; i++)
        {
            while (case_data[id + sub_id] != ',') //每个障碍物顶点个数
            {
                sub_id++;
            }
            obstacle_vertex_num.push_back(atof(case_data.substr(id, sub_id).c_str()));
            id += (++sub_id);
            sub_id = 0;
        }
        std::vector<Point> obstacle_vertex;
        Point vertex;
        for (int i = 0; i < obstacle_num; i++)
        {
            obstacle_vertex.clear();
            for (int j = 0; j < obstacle_vertex_num[i]; j++)
            {
                while (case_data[id + sub_id] != ',') //障碍物顶点x坐标
                {
                    sub_id++;
                }
                vertex.x = atof(case_data.substr(id, sub_id).c_str());
                id += (++sub_id);
                sub_id = 0;
                while (case_data[id + sub_id] != ',') //障碍物顶点y坐标
                {
                    sub_id++;
                    if (id + sub_id == case_data.size()) // csv 文件最后一个数据无‘,’，需要判断是否退出循环
                        break;
                }
                vertex.y = atof(case_data.substr(id, sub_id).c_str());
                id += (++sub_id);
                sub_id = 0;
                obstacle_vertex.push_back(vertex);
            }
            map_info.obstacles.push_back(obstacle_vertex);
        }
        std::vector<double> vertex_x, vertex_y;
        for (int i = 0; i < map_info.obstacles.size(); i++)
        {
            for (int j = 0; j < map_info.obstacles[i].size(); j++)
            {
                vertex_x.push_back(map_info.obstacles[i][j].x);
                vertex_y.push_back(map_info.obstacles[i][j].y);
            }
        }
        double minX = *std::min_element(vertex_x.begin(), vertex_x.end());
        double maxX = *std::max_element(vertex_x.begin(), vertex_x.end());
        double minY = *std::min_element(vertex_y.begin(), vertex_y.end());
        double maxY = *std::max_element(vertex_y.begin(), vertex_y.end());
        minX = std::min(minX, std::min(start.x, goal.x));
        maxX = std::max(maxX, std::max(start.x, goal.x));
        minY = std::min(minY, std::min(start.y, goal.y));
        maxY = std::max(maxY, std::max(start.y, goal.y));
        map_info.corrd_shift = {minX - padding / 2, minY - padding / 2};
        for (int i = 0; i < map_info.obstacles.size(); i++) //更新地图偏移后的顶点坐标
        {
            for (int j = 0; j < map_info.obstacles[i].size(); j++)
            {
                map_info.obstacles[i][j].x -= map_info.corrd_shift.x;
                map_info.obstacles[i][j].y -= map_info.corrd_shift.y;
            }
        }
        map_info.width = std::ceil((maxX + padding / 2 - map_info.corrd_shift.x) / map_info.resolution);
        map_info.heigth = std::ceil((maxY + padding / 2 - map_info.corrd_shift.y) / map_info.resolution);

        return true;
    }

    bool CsvToGridMap::getGridMap(GridMap &grid_map)
    {
        if (!parseCsv())
        {
            return false;
        }
        grid_map.cols = map_info.width;
        grid_map.rows = map_info.heigth;
        grid_map.resolution = map_info.resolution;
        grid_map.occs.resize(grid_map.cols);
        for (int i = 0; i < grid_map.cols; i++)
        {
            grid_map.occs[i].resize(grid_map.rows);
            for (int j = 0; j < grid_map.rows; j++)
            {
                grid_map.occs[i][j] = false;
            }
        }
        std::vector<intPoint> polygon_area;
        for (int i = 0; i < map_info.obstacles.size(); i++)
        {
            fillPolygon(map_info.obstacles[i], map_info.resolution, polygon_area);
        }
        for (int i = 0; i < polygon_area.size(); i++)
        {
            grid_map.occs[polygon_area[i].x][polygon_area[i].y] = true;
        }
        return true;
    }
}