//
// Created by yt on 10/31/17.
//

#ifndef PATH_PLANNING_CG_SOLVER_H
#define PATH_PLANNING_CG_SOLVER_H

#include <vector>
#include <cppad/cppad.hpp>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

class CG_Solver {
 public:
    std::vector< double > Xk;
    std::vector< double > X0;
    std::vector< double > Xn;
    CG_Solver(std::vector<geometry_msgs::Point> &path);
    CG_Solver(const nav_msgs::Path &path,
              double w1,
              double w2,
              double w3,
              bool boundary,
              const grid_map::GridMap &gridmap);
    CG_Solver();
    CG_Solver(const nav_msgs::Path &path,
              const grid_map::GridMap &dis_map,
              const grid_map::GridMap &grad_map);
    CG_Solver(const nav_msgs::Path &path);
    void Solve();

    nav_msgs::Path getSmoothPath();

 private:
    double w1_{5.0};
    double w2_{10.0};
    double w3_{10.0};
    int numPoint;
    bool is_consider_boundary_{true};
    grid_map::GridMap gridmap_;
    grid_map::GridMap map_grad;

    CppAD::ADFun<double> Fun;

    double stepLineSearch(const std::vector<double> &X,
                          const std::vector<double> &d,
                          const std::vector<double> &g,
                          double F);
    double amijoSearch(const std::vector<double> &X,
                                     const std::vector<double> &d,
                                     const std::vector<double> &g,
                                     double F);
    std::vector<double> getFuncValue(const std::vector<double> &x,
                                      const std::vector<double> &x0,
                                      const std::vector<double> &xn);
    void getGradient(const std::vector<double> &x, std::vector<double> *g);


///CppAD function
    CppAD::AD<double> acos_(const CppAD::AD<double> &x) {
            if(x >= 1.0)
                return 0.0;
        return x.acos_me();
    }

    CppAD::AD<double> sqrt_(const CppAD::AD<double> &x) {
        return x.sqrt_me();
    }

    double acos_(const double &x){
        if(x >= 1.0)
            return 0.0;
        return std::acos(x);
    }
    double sqrt_(const double &x){
        return std::sqrt(x);
    }

    template <class Type>
    Type dot_(Type x1, Type y1, Type x2, Type y2){
        return x1 * x2 + y1 * y2;
    }

    template <class Type>
    Type norm_(Type x1, Type y1){
        return x1 * x1 + y1 * y1;
    }

    template <class Type>
    double dot_(const Type &X1,const Type &X2){
        double r = 0;
        for(int i=0; i<X1.size(); i++){
            r += X1[i] * X2[i];
        }
        return r;
    }

    template <class Type>
    Type multi_(double a, const Type &X){
        Type r(X.size());
        for(int i=0; i<X.size(); i++){
            r[i] = X[i] * a;
        }
        return r;
    }

    template <class Type>
    Type add_(const Type &X1,const Type &X2){
        Type r(X1.size());
        for(int i=0; i<X1.size(); i++){
            r[i] = X1[i] + X2[i];
        }
        return r;
    }

    template <class Type>
    Type Poly2( const Type &x, const Type &x0, const Type &xn)
    {
        Type y(1)   ;//= 0.;  // initialize summation
        y[0] = 0.;
        size_t numPoint = x.size() / 2;
        Type dx(numPoint+1);
        Type dy(numPoint+1);
        Type dTheta(numPoint);
        Type dist(numPoint);
        for(int i=0; i< numPoint + 1; i++){
            if(i == 0) {
                dx[i] = x[2*i] - x0[0];
                dy[i] = x[2*i+1] - x0[1];
            }
            else if(i == numPoint)
            {
                dx[i] = xn[0] - x[2*(i-1)];
                dy[i] = xn[1] - x[2*(i-1)+1];
            }
            else
            {
                dx[i] = x[2*i] - x[2*(i-1)];
                dy[i] = x[2*i+1] - x[2*(i-1)+1];
            }
            if(i > 0 ){
                auto dot = dot_(dx[i-1], dy[i-1], dx[i], dy[i]);
                auto norm1 = sqrt_(norm_(dx[i-1], dy[i-1]));
                auto norm2 = sqrt_(norm_(dx[i], dy[i]));
                dTheta[i-1] = acos_(dot/norm1/norm2);

                y[0] += this->w1_*norm_(dx[i] - dx[i-1], dy[i] - dy[i-1]);
                y[0] += this->w2_*dTheta[i-1]*dTheta[i-1]/norm_(dx[i-1], dy[i-1]);
            }
            if(i>50){
                int a =10;
            }
        }
        return y;
    }
    template <class Type>
    Type Poly( const Type &x, const Type &x0, const Type &xn)
    {
        Type y(1)   ;//= 0.;  // initialize summation
        y[0] = 0.;
        size_t numPoint = x.size() / 2;
        Type dx(numPoint+1);
        Type dy(numPoint+1);
        Type dTheta(numPoint);
        Type dist(numPoint);
        for (int i = 0; i < numPoint + 1; i++) {
            if (i == 0) {
                dx[i] = x[2*i] - x0[0];
                dy[i] = x[2*i+1] - x0[1];
            } else if (i == numPoint) {
                dx[i] = xn[0] - x[2*(i-1)];
                dy[i] = xn[1] - x[2*(i-1)+1];
            } else {
                dx[i] = x[2*i] - x[2*(i-1)];
                dy[i] = x[2*i+1] - x[2*(i-1)+1];
            }
            if(i > 0 ) {
                auto dot = dot_(dx[i-1], dy[i-1], dx[i], dy[i]);
                auto norm1 = sqrt_(norm_(dx[i-1], dy[i-1]));
                auto norm2 = sqrt_(norm_(dx[i], dy[i]));
                dTheta[i-1] = acos_(dot/norm1/norm2);

                y[0] += this->w1_*norm_(dx[i] - dx[i-1], dy[i] - dy[i-1]);
                y[0] += this->w2_*dTheta[i-1]*dTheta[i-1]/norm_(dx[i-1], dy[i-1]);
            }
        }
        int a = 50;
        return y;
    }
};

#endif //PATH_PLANNING_CG_SOLVER_H
