#include "MPC.h"
#include "Solution.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

#define TUNING_1 10.0     // Reference Velocity
#define TUNING_2 3000.0   // Euclidean Distance
#define TUNING_3 150.0    // State Error
#define TUNING_4 2000.0   // Starting Error
#define TUNING_5 150.0    // Steering Rate Penalty
#define TUNING_6 20.0     // Acceleration Rate Penalty

using CppAD::AD;

// Simulator parameters.
size_t N = 12;
double dt = 0.05;
const double Lf = 2.67;
double ref_v = 100;

// MPC Indexing Parameters.
unsigned int x_start = 0;
unsigned int y_start = x_start + N;
unsigned int psi_start = y_start + N;
unsigned int v_start = psi_start + N;
unsigned int cte_start = v_start + N;
unsigned int epsi_start = cte_start + N;
unsigned int delta_start = epsi_start + N;
unsigned int a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    // Reset the cost function.
    fg[0] = 0;

    // Implement Cost Functions.
    for (unsigned int t = 0; t < N; t++) {

      // Cost Function 1: Reference Velocity.
      fg[0] += TUNING_1 * CppAD::pow(vars[v_start + t] - ref_v, 2);

      // Cost Function 2: Initial State Error.
      fg[0] += TUNING_2 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += TUNING_2 * CppAD::pow(vars[epsi_start + t], 2);
    }

    // Cost Function 3: Actuator Response.
    for (unsigned int t = 0; t < N - 1; t++) {
      fg[0] += TUNING_3 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += TUNING_3 * CppAD::pow(vars[a_start + t], 2);
      fg[0] += TUNING_4 * CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; t++) {
      fg[0] += TUNING_5 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += TUNING_6 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (unsigned int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      AD<double> delta = vars[delta_start + t - 1];
      if (t > 1) {
        a0 = vars[a_start + t - 2];
        delta = vars[delta_start + t - 2];
      }
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);
    }
  }
};


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

//vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Extract initial state for readability.
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set initial variables.
  unsigned int n_vars = 6 * N + 2 * (N - 1);
  unsigned int n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Set lower and upper limits.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  for (unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //std::cout << "Cost " << solution.obj_value << std::endl;

  // Assemble result to return
  Solution result;
  result.steer = solution.x[delta_start];
  result.throttle = solution.x[a_start];

  for (unsigned int i = 1; i < N-1; i++) {
    result.xvals.push_back(solution.x[x_start + i + 1]);
    result.yvals.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
