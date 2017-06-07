#include "MPC.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>

using CppAD::AD;

class FG_eval {
public:
    Eigen::VectorXd coeffs;

    // Coefficients of the fitted polynomial.
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

//     `fg` is a vector containing the cost and constraints.
//     `vars` is a vector containing the variable values (state & actuators).
    void operator()(ADvector &fg, const ADvector &vars) {
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (int i = 0; i < N; i++) {
            fg[0] += LAMBDA_CTE * CppAD::pow(vars[IX_CTE_START + i] - REF_CTE, 2);
            fg[0] += LAMBDA_EPSI * CppAD::pow(vars[IX_EPSI_START + i] - REF_EPSI, 2);
            fg[0] += LAMBDA_V * CppAD::pow(vars[IX_V_START + i] - REF_V, 2);
        }

        // Minimize the use of actuators.
        for (int i = 0; i < N - 1; i++) {
            fg[0] += LAMBDA_DELTA * CppAD::pow(vars[IX_DELTA_START + i], 2);
            fg[0] += LAMBDA_A * CppAD::pow(vars[IX_A_START + i], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < N - 2; i++) {
            fg[0] += LAMBDA_DDELTA * CppAD::pow(vars[IX_DELTA_START + i + 1] - vars[IX_DELTA_START + i], 2);
            fg[0] += LAMBDA_DA * CppAD::pow(vars[IX_A_START + i + 1] - vars[IX_A_START + i], 2);
        }


        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + IX_X_START] = vars[IX_X_START];
        fg[1 + IX_Y_START] = vars[IX_Y_START];
        fg[1 + IX_PSI_START] = vars[IX_PSI_START];
        fg[1 + IX_V_START] = vars[IX_V_START];
        fg[1 + IX_CTE_START] = vars[IX_CTE_START];
        fg[1 + IX_EPSI_START] = vars[IX_EPSI_START];

        // The rest of the constraints
        for (int i = 0; i < N - 1; i++) {

            // The state at time t.
            AD<double> x0 = vars[IX_X_START + i];
            AD<double> y0 = vars[IX_Y_START + i];
            AD<double> psi0 = vars[IX_PSI_START + i];
            AD<double> v0 = vars[IX_V_START + i];
            AD<double> cte0 = vars[IX_CTE_START + i];
            AD<double> epsi0 = vars[IX_EPSI_START + i];


            // The state at time t+1 .
            AD<double> x1 = vars[IX_X_START + i + 1];
            AD<double> y1 = vars[IX_Y_START + i + 1];
            AD<double> psi1 = vars[IX_PSI_START + i + 1];
            AD<double> v1 = vars[IX_V_START + i + 1];
            AD<double> cte1 = vars[IX_CTE_START + i + 1];
            AD<double> epsi1 = vars[IX_EPSI_START + i + 1];


            // Only consider the actuation at time t.
            AD<double> delta0 = vars[IX_DELTA_START + i];
            AD<double> a0 = vars[IX_A_START + i];


            AD<double> f0 = 0.0;
            for (int i = 0; i < coeffs.size(); i++) {
                f0 += coeffs[i] * pow(x0, i);
            }
            AD<double> psides0 = 0.0;
            for (int i = 1; i < coeffs.size(); i++) {
                psides0 += i * coeffs[i] * pow(x0, i - 1);
            }
            psides0 = CppAD::atan(psides0);


            fg[2 + IX_X_START + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
            fg[2 + IX_Y_START + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
            fg[2 + IX_PSI_START + i] = psi1 - (psi0 + v0 * (-delta0 / LF) * DT);
            fg[2 + IX_V_START + i] = v1 - (v0 + a0 * DT);
            fg[2 + IX_CTE_START + i] =
                    cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
            fg[2 + IX_EPSI_START + i] =
                    epsi1 - ((psi0 - psides0) + v0 * (-delta0 / LF) * DT);
        }
    }


};

//
// MPC class definition
//

MPC::MPC() {
    steer = 0;
    throttle = 0;
}

MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = x0[0];
    double y = x0[1];
    double psi = x0[2];
    double v = x0[3];
    double cte = x0[4];
    double epsi = x0[5];

    // number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = N * 6 + (N - 1) * 2;
    // Number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }
    // Set the initial variable values
    vars[IX_X_START] = x;
    vars[IX_Y_START] = y;
    vars[IX_PSI_START] = psi;
    vars[IX_V_START] = v;
    vars[IX_CTE_START] = cte;
    vars[IX_EPSI_START] = epsi;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < IX_DELTA_START; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (int i = IX_DELTA_START; i < IX_A_START; i++) {
        vars_lowerbound[i] = -deg2rad(25);
        vars_upperbound[i] = deg2rad(25);
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (int i = IX_A_START; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[IX_X_START] = x;
    constraints_lowerbound[IX_Y_START] = y;
    constraints_lowerbound[IX_PSI_START] = psi;
    constraints_lowerbound[IX_V_START] = v;
    constraints_lowerbound[IX_CTE_START] = cte;
    constraints_lowerbound[IX_EPSI_START] = epsi;

    constraints_upperbound[IX_X_START] = x;
    constraints_upperbound[IX_Y_START] = y;
    constraints_upperbound[IX_PSI_START] = psi;
    constraints_upperbound[IX_V_START] = v;
    constraints_upperbound[IX_CTE_START] = cte;
    constraints_upperbound[IX_EPSI_START] = epsi;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    //
    // Check some of the solution values
    //
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    if (ok) {
        last_solution = solution.x;
        steps_since_last_solution = 0;

        x_vals = {};
        y_vals = {};
        for (int i = 0; i < N; i++) {
            x_vals.push_back(solution.x[IX_X_START + i]);
            y_vals.push_back(solution.x[IX_Y_START + i]);
        }

        auto cost = solution.obj_value;
        std::cout << "Solution successful; Cost " << cost << std::endl;
        steer = last_solution[IX_DELTA_START];
        throttle = last_solution[IX_A_START];
    } else if (steps_since_last_solution < N) {
        std::cout << "Solution unsuccessful; Using last valid from " << steps_since_last_solution << " steps ago"
                  << std::endl;
        steps_since_last_solution++;
        steer = last_solution[IX_DELTA_START + steps_since_last_solution];
        throttle = last_solution[IX_A_START + steps_since_last_solution];
    } else {
        std::cout << "Solution unsuccessful and no valid action left" << std::endl;
        steps_since_last_solution++;
        steer = 0;
        throttle = 0;
    }
}