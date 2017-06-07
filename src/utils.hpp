#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#ifndef MPC_UTILS_HPP
#define MPC_UTILS_HPP

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

static double deg2rad(double x) { return x * pi() / 180; }

static double rad2deg(double x) { return x * 180 / pi(); }

static double mph2mps(double mph) {
    return mph * 0.44704;
}


struct State {
    double x;
    double y;
    double psi;
    double v;
};


static State globalKinematic(State &state,
                             double delta,
                             double a,
                             double dt,
                             double lf) {

    auto nextX = state.x + state.v * cos(state.psi) * dt;
    auto nextY = state.y + state.v * sin(state.psi) * dt;
    auto nextPsi = state.psi + (state.v * -delta / lf) * dt;
    auto nextV = state.v + a * dt;

    State nextState = {nextX, nextY, nextPsi, nextV};
    return nextState;
}


// Evaluate a polynomial.
static double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
static Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                               int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


static std::tuple<std::vector<double>, std::vector<double>>
createWaypointVizualization(Eigen::VectorXd poly, int nb_vizpoints, double step_size) {
    std::vector<double> vizpoints_x(nb_vizpoints);
    std::vector<double> vizpoints_y(nb_vizpoints);

    for (int i = 0; i < nb_vizpoints; ++i) {

        double dx = step_size * i;
        double dy = polyeval(poly, dx);

        vizpoints_x[i] = dx;
        vizpoints_y[i] = dy;
    }

    return std::make_tuple(vizpoints_x, vizpoints_y);
}

#endif //MPC_UTILS_HPP
