#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

/**
 * Set the timestep length and duration
 */
size_t N = 10;
double dt = 100.0/1000.0; // milliseconds

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const double ref_cte = 0;
const double ref_epsi = 0;
const double ref_v = 100;

const size_t x_address = 0;
const size_t y_address = N;
const size_t psi_address = 2*N;
const size_t v_address = 3*N;
const size_t cte_address = 4*N;
const size_t epsi_address = 5*N;
const size_t delta_address = 6*N;
const size_t a_address = 7*N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable 
     *   values (state & actuators)
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */

    fg[0] = 0;

    for( int i = 0; i < N; i++ ) {
      fg[0] += 1000*CppAD::pow(vars[cte_address + i] - ref_cte, 2);
      fg[0] += 1000*CppAD::pow(vars[epsi_address + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_address + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i< N - 1; i++) {
      fg[0] += 50*CppAD::pow(vars[delta_address + i], 2);
      fg[0] += 50*CppAD::pow(vars[a_address + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    // (how smooth the actuations are)
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 250000*CppAD::pow(vars[delta_address + i + 1] - vars[delta_address + i], 2);
      fg[0] += 5000*CppAD::pow(vars[a_address + i + 1] - vars[a_address + i], 2);
    }

    // Initial constraints.
    fg[1 + x_address] = vars[x_address];
    fg[1 + y_address] = vars[y_address];
    fg[1 + psi_address] = vars[psi_address];
    fg[1 + v_address] = vars[v_address];
    fg[1 + cte_address] = vars[cte_address];
    fg[1 + epsi_address] = vars[epsi_address];

    for (int t = 1; t < N; t++) {
      // The state at time t+1
      AD<double> x1 = vars[x_address + t];
      AD<double> y1 = vars[y_address + t];
      AD<double> psi1 = vars[psi_address + t];
      AD<double> v1 = vars[v_address + t];
      AD<double> cte1 = vars[cte_address + t];
      AD<double> epsi1 = vars[epsi_address + t];

      // The state at time t.
      AD<double> x0 = vars[x_address + t - 1];
      AD<double> y0 = vars[y_address + t - 1];
      AD<double> psi0 = vars[psi_address + t - 1];
      AD<double> v0 = vars[v_address + t - 1];
      AD<double> cte0 = vars[cte_address + t - 1];
      AD<double> epsi0 = vars[epsi_address + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_address + t - 1];
      AD<double> a0 = vars[a_address + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      
      // The equations for the Kinematic model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      
      fg[1 + x_address + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_address + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_address + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
      fg[1 + v_address + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_address + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_address + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta0 * dt);
    }
  }
};

// MPC class definition implementation.

MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  /**
   * Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars = 6*N + 2*(N-1);
  /**
   * Set the number of constraints
   */
  size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /*
    Set lower and upper limits for variables.
  */

  // Setting all non-actuators upper and lower limits to max
  // negative and positive values
  for (int i=0; i< delta_address; i++){
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Steering limits are -25 to 25 deg.
  for ( int i = delta_address; i < a_address; i++ ) {
    // converting degrees to radians
    vars_lowerbound[i] = -25*M_PI/180;
    vars_upperbound[i] = 25*M_PI/180;
  }

  // Throttle limits are -1 to 1
  for ( int i = a_address; i < n_vars; i++ ) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_address] = x;
  constraints_lowerbound[y_address] = y;
  constraints_lowerbound[psi_address] = psi;
  constraints_lowerbound[v_address] = v;
  constraints_lowerbound[cte_address] = cte;
  constraints_lowerbound[epsi_address] = epsi;

  constraints_upperbound[x_address] = x;
  constraints_upperbound[y_address] = y;
  constraints_upperbound[psi_address] = psi;
  constraints_upperbound[v_address] = v;
  constraints_upperbound[cte_address] = cte;
  constraints_upperbound[epsi_address] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */
  std::vector<double> result;

  result.push_back(solution.x[delta_address]);
  result.push_back(solution.x[a_address]);

  for ( int i = 0; i < N - 2; i++ ) {
    result.push_back(solution.x[x_address + i + 1]);
    result.push_back(solution.x[y_address + i + 1]);
  }
  return result;
}