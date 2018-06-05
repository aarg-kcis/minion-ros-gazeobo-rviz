#ifndef OCPSOLVER_H
#define OCPSOLVER_H

#include <Eigen/Dense>
#include <iostream>
extern "C"{
#include "solver.h"
}



class MPCsolver
{

public:
    MPCsolver();

    // Design a trajectory containing desired position, velocity, acceleration, jerk via the solution of the convex optimization problem
    Eigen::Matrix<double, 10, 17> OCPsolDesigner(double deltaT, Eigen::Matrix<double, 3, 16> obstacle_force, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 16, 1> curState,  Eigen::Matrix<double, 16, 1> termState, Eigen::Matrix<double, 21, 1> CostFunction, Eigen::Matrix<double, 16, 2> stateLimits, Eigen::Matrix<double, 4, 2> input_constraint, std::vector<double> mates_Yaw, std::vector<int>theta_cost_weights);
    
    // Load and generate required parameters for the optimal control problem
	void load_data(double deltaT, Eigen::Matrix<double, 3, 16> obstacle_force, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 16, 1> InitialState, Eigen::Matrix<double, 16, 1> TerminalState, Eigen::Matrix<double, 16, 2> StateConstraint, Eigen::Matrix<double, 4, 2> InputConstraint, Eigen::Matrix<double, 21, 1> CostFunction, std::vector<double> mates_Yaw, std::vector<int>theta_cost_weights);
    
    // output the solution of decoupled optimal control problems
    // x vx, y,vy,z,vz for 16 time steps and one more field for latest control inputs.
    Eigen::Matrix<double, 10, 17> use_solution(Vars vars);

};

#endif // OCPSOLVER_H
