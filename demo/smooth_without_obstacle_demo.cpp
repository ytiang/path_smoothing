//
// Created by yangt on 19-2-18.
//
#include "path_smoothing/path_smoothing.hpp"
#include "csv_reader.hpp"

#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <fstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_smooth_demo");
    std::string basic_dir = ros::package::getPath("path_smoothing");
    io::CSVReader<2> in(basic_dir + "/demo/path.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    std::vector<geometry_msgs::Point> path;

    geometry_msgs::Point pt;
    while (in.read_row(pt.x, pt.y)) {
        path.push_back(pt);
    }

    using namespace path_smoothing;

    PathSmoothing::Options options;
    options.cg_solver = SELF_SOLVER;
//    options.type = CASADI;
    std::unique_ptr<PathSmoothing>
            smoother(PathSmoothing::createSmoother(options, path));
    smoother->smoothPath(options);
    smoother->getSmoothPath(&path);


    // write to file:
    std::ofstream fout(basic_dir+"/demo/smooth_without_obstacle_result.csv");
    if(fout.is_open()) {
        fout << "\"x\"" << "," << "\"y\"" << "\n";
        for (int i(0); i < path.size(); ++i) {
            fout << path.at(i).x << "," << path.at(i).y << "\n";
        }
        fout.close();
    }

}

