#include <definitions.h>

dxdt::dxdt(double tau_, double ts_, int order_) : tau(tau_), ts(ts_), order(order_)
{
    a1 = (2 * tau - ts) / (2 * tau + ts);
    a2 = 2 / (2 * tau + ts);
    dot = Vector3d(0, 0, 0);
    x_d1 = Vector3d(0, 0, 0);
}

dxdt::dxdt() {}

Vector3d dxdt::calculate(const Vector3d &x)
{
    if (it > order)
    {
        dot = a1 * dot + a2 * (x - x_d1);
    }
    it++;
    x_d1 = x;
    return dot;
}
