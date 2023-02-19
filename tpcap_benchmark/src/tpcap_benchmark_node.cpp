#include "../include/tpcap_benchmark/csv_to_occmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tpcap_benchmark_node");
    
    TPCAP::CsvToOccmap cto;
    while(ros::ok())
    {
        cto.pubMapAndShift();
    }

    return 0;
}