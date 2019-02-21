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

    ncopt::Vector points((path.size() - 2) * 2);
    for (int i = 0; i < path.size() - 2; ++i) {
        points(i * 2) = path.at(i).x;
        points(i * 2 + 1) = path.at(i).y;
    }

    using namespace path_smoothing;
    PathSmoothing::Options options;
    options.type = CASADI;
    auto t1 = hmpl::now();
    PathSmoothing smoother(path, options);
    ncopt::GradientProblemOption solver_options;
    ncopt::Summary summarys;

    ncopt::GradientProblemSolver solver(&smoother);
    auto t2 = hmpl::now();
    solver.Solve(points.data(), solver_options, &summarys);
    auto t3 = hmpl::now();
    ncopt::Summary::PrintSummary(summarys);
    printf("initilization time: %f, solve time: %f\n", hmpl::getDurationInSecs(t1, t2), hmpl::getDurationInSecs(t2, t3));

//    Vector start(2), end(2);
//    int i = 0;
//    double x, y;
//    in.read_row(x, y);
//    start << x, y;
//    while (in.read_row(x, y)) {
//        // do stuff with the data
//        temp_points.push_back(x);
//        temp_points.push_back(y);
//    }
//    end << x, y;
//    VectorRef points(temp_points.data(), temp_points.size() - 2);
//    Vector gradient(points.size());
//    printf("points size: %d\n", points.size() / 2);
//    SmoothingCostFunction smoother(points.size(), 2);
//    smoother.SetEndPoints(start, end);
//    GradientProblemOption options;
//    Summary summary;
//    timeval ts;
//    gettimeofday(&ts, NULL);
//    GradientProblemSolver solver(&smoother);
//    solver.Solve(points.data(), options, &summary);
//    timeval te;
//    gettimeofday(&te, NULL);
//    Summary::PrintSummary(summary);
//    printf("time cost: %f\n", GetTimeInteral(te, ts));
//
//    // write to file:
    hmpl::CSVWriter writer(",");
    writer.newRow() << "x" << "y";
    writer.newRow() << path.front().x << path.front().y;
    for (int i(0); i < points.size() / 2; ++i) {
        writer.newRow() << points(i * 2) << points(i * 2 + 1);
    }
    writer.newRow() << path.back().x << path.back().y;
    writer.writeToFile("/home/yangt/smooth.csv");

}

