//
// Created by yangt on 19-2-18.
//
#include "path_smoothing/path_smoothing.hpp"
#include "opt_utils/csv_reader.hpp"
#include "opt_utils/opt_utils.hpp"
#include <geometry_msgs/Point.h>
#include "opt_utils/csv_writer.hpp"
#include <ros/package.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_smooth_demo");
    std::string basic_dir = ros::package::getPath("path_smoothing");
    io::CSVReader<2> in(basic_dir + "/test/path.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    std::vector<geometry_msgs::Point> path;

    geometry_msgs::Point pt;
    while (in.read_row(pt.x, pt.y)) {
        path.push_back(pt);
    }

    using namespace path_smoothing;
    PathSmoothing::Options options;
//    options.type = CASADI;
    auto t1 = hmpl::now();
    PathSmoothing *smoother = PathSmoothing::createSmoother(options, &path);
    smoother->smoothPath(options);
    auto t2 = hmpl::now();
    printf("smooth time: %f\n", hmpl::getDurationInSecs(t1, t2));
    smoother->getPointPath(&path);

    // write to file:
    hmpl::CSVWriter writer(",");
    writer.newRow() << "x" << "y";
    for (int i(0); i < path.size(); ++i) {
        writer.newRow() << path.at(i).x << path.at(i).y;
    }
    writer.writeToFile(basic_dir + "/test/smooth_result.csv");

}

