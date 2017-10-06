#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    if (estimations.size() != ground_truth.size() || estimations.size() < 1) {
        throw std::runtime_error("Input sizes are incorrect!");
    }

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    //accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        // ... your code here
        for (size_t j = 0; j < 4; ++j) {
            double diff = estimations[i][j] - ground_truth[i][j];
            rmse[j] += (diff * diff);
        }
    }

    for (size_t j = 0; j < 4; ++j) {
        rmse[j] = rmse[j] / estimations.size();
    }
    for (size_t j = 0; j < 4; ++j) {
        rmse[j] = sqrt(rmse[j]);
    }

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    if (x_state.size() != 4) {
        throw std::runtime_error("Jacobian can only be calculated for a state of size 4!");
    }
    MatrixXd Hj(3, 4);
    Hj.setZero();
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE

    double one = px * px + py * py;
    double two = sqrt(one);
    double three = one * two;

    //check division by zero
    if (one < 0.00001) {
        std::cout << "Division by zero." << std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << px / two, py / two, 0, 0,
            -py / one, px / one, 0, 0,
            py * (vx * py - vy * px) / three, px * (vy * px - vx * py) / three, px / two, py / two;

//  std::cout << Hj << std::endl;
    return Hj;
}
