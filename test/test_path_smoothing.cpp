//
// Created by yangt on 19-2-18.
//
#include "path_smoothing/path_smoothing.hpp"
#include "opt_utils/csv_reader.hpp"
#include "opt_utils/opt_utils.hpp"
#include <geometry_msgs/Point.h>
#include "opt_utils/csv_writer.hpp"
#include <ros/package.h>

int main() {
    io::CSVReader<2> in
            ("/home/yangt/workspace/ros_ws/catkin_ws/src/path_smoothing/path.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    std::vector<geometry_msgs::Point> path;

    geometry_msgs::Point pt;
    while (in.read_row(pt.x, pt.y)) {
        path.push_back(pt);
    }

    using namespace path_smoothing;
    PathSmoothing::Options options;
    auto t1 = hmpl::now();
    PathSmoothing smoother;
    smoother.smoothPath(options, &path);
    auto t2 = hmpl::now();
    printf("smooth time: %f\n",
           hmpl::getDurationInSecs(t1, t2));

    // write to file:
    hmpl::CSVWriter writer(",");
    writer.newRow() << "x" << "y";
    for (int i(0); i < path.size(); ++i) {
        writer.newRow() << path.at(i).x << path.at(i).y;
    }
    writer.writeToFile("/home/yangt/smooth.csv");

}

