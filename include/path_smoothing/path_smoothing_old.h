#ifndef PATH_PLANNING_PATH_SMOOTHING_H
#define PATH_PLANNING_PATH_SMOOTHING_H

#include "ceres/ceres.h"
#include <nav_msgs/Path.h>
#include <cppad/cppad.hpp>
#include <geometry_msgs/Point.h>
#include <internal_grid_map/internal_grid_map.hpp>

class PathSmooth : public ceres::FirstOrderFunction {
 public:
    double *Xi;
    PathSmooth(const std::vector<geometry_msgs::Point> &path );
    PathSmooth(const nav_msgs::Path &path,
               double w1,
               double w2,
               double w3,
               bool boundary,
               const grid_map::GridMap &map);
    PathSmooth(const nav_msgs::Path &path, 
               const grid_map::GridMap &distance_map, 
               const grid_map::GridMap &grad_map);
    ~PathSmooth(){
        delete Fun_;
    }
    nav_msgs::Path getSmoothPath();
    void setParams(double w1, double w2, double w3);
 private:
    int numParam_;
    std::vector<double> X_init_;
    std::vector<double> X_goal_;
    double w1_;
    double w2_;
    double w3_;
    CppAD::ADFun<double> *Fun_;
    grid_map::GridMap gridmap_;
    grid_map::GridMap gradient_map_;
    bool is_consider_boundary_;


    virtual bool Evaluate(const double* parameters,
                          double* cost,
                          double* gradient) const;

    virtual int NumParameters() const ;

    CppAD::AD<double> acos_(const CppAD::AD<double> &x) const{
        if(x > 1.0)
            return 0.;
        return x.acos_me();
    }

    CppAD::AD<double> sqrt_(const CppAD::AD<double> &x) const {
        return x.sqrt_me();
    }

    double acos_(const double &x) const{
        if(x > 1.0)
            return 0.;
        return std::acos(x);
    }
    double sqrt_(const double &x) const{
        return std::sqrt(x);
    }


    template <class Type>
    Type dot_(Type x1, Type y1, Type x2, Type y2) const{
        return x1 * x2 + y1 * y2;
    }

    template <class Type>
    Type norm_(Type x1, Type y1) const{
        return x1 * x1 + y1 * y1;
    }

    template <class Type>
    double dot_(const Type &X1,const Type &X2) const{
        double r = 0;
        for(int i=0; i<X1.size(); i++){
            r += X1[i] * X2[i];
        }
        return r;
    }
      
    template <class Type>
    Type Poly( const Type &x, const Type &x0, const Type &xn) const
    {

        Type y(1)   ;//= 0.;  // initialize summation
        y[0] = 0.;
        size_t numPoint = x.size() / 2;
        Type dx(numPoint+1);
        Type dy(numPoint+1);
        Type dTheta(numPoint);
        for(int i=0; i< numPoint + 1; i++){
            if(i == 0){
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
        }
        return y;
    }
};


#endif //PATH_PLANNING_PATH_SMOOTHING_H

