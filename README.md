# Path Smoothing
this package is for path_smoothing. Two methods are developed in this repo, including [conjugate gradient method](http://xueshu.baidu.com/usercenter/paper/show?paperid=b5778bb39c6db872ad2e507f3a14b23f&site=xueshu_se) and [Gauss Process method](https://github.com/gtrll/gpmp2). The latter considers vehicle dynamics but is unstable now.

## Dependency
* [ceres-solver](https://github.com/ceres-solver/ceres-solver) is required for solving Conjugate Gradient method.
* [cppad](https://coin-or.github.io/CppAD/doc/cppad.htm) and [casadi](https://github.com/casadi/casadi) are two necessary Automatic  Differentiation tools.
* [grid_map](https://github.com/ANYbotics/grid_map) is necessary for obstacle consideration.
* [gtsam4.0](https://bitbucket.org/gtborg/gtsam), [gpmp2](https://github.com/gtrll/gpmp2) are optional, unless you want to use Gauss Process method.
* [opt_utils](https://github.com/bit-ivrc/hmpl/tree/master/opt_utils), [rosparam_handler](https://github.com/cbandera/rosparam_handler), [internal_grid_map](https://github.com/bit-ivrc/hmpl/tree/master/internal_grid_map), [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) are used ros package in demo.

## Test
* test path smoothing without obstacle
````
rosrun path smoothing without obstacle
cd <path_smoothing-package-dir>/demo
python simple_curve_plot.py
````
* test path smoothing with obstacle
````
roslaunch path_smoothing smooth_with_obstacle_demo.launch
````
## How to use
Please reference provided two demos: [smooth_without_obstacle_demo](./demo/smooth_without_obstacle_demo.cpp) and [smooth_with_obstacle_demo](./demo/smooth_with_obstacle_demo.cpp).

## Next
add gradient-free method in hmpl.
