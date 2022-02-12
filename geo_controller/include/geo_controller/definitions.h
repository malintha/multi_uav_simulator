#include <eigen3/Eigen/Dense>

using namespace Eigen;

// compute the derivative of a signal
struct dxdt {
    double tau;
    double ts;
    double a1;
    double a2;
    int order;
    Vector3d dot;
    Vector3d x_d1;
    int it = 1;

    dxdt();
    dxdt(double tau_, double ts_, int order_);
    Vector3d calculate(const Vector3d &x);
};

typedef struct
{
    Vector3d ex;
    Vector3d ev;
    Vector3d ea;
    Vector3d ej;
    Vector3d er;
    Vector3d eomega;
} err_t;

typedef struct {
    // derivatives of desired states
    Vector3d d_xd;
    Vector3d d2_xd;
    Vector3d d3_xd;
    Vector3d d4_xd;
    Vector3d db1;
    Vector3d d2b1;

    // derivatives of current velocity
    Vector3d dv;
    Vector3d d2v;

} derivatives_t;