#pragma once

#include <Eigen/Dense>
#include "Control/control.hpp"

namespace Control {

FilterError tustin_1_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FilterError tustin_2_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FilterError tustin_n_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);

FilterError tf2ss(const FiltVectorXf &num, const FiltVectorXf &den, 
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, NUM_STATES, NUM_STATES> &A, 
            Eigen::Matrix<float, Eigen::Dynamic, 1, NUM_STATES, 1> &B, 
            Eigen::Matrix<float, 1, Eigen::Dynamic, 1, NUM_STATES> &C, 
            Eigen::Matrix<float, 1, 1, 1, 1> &D);

FilterError tustin_2_tf(const Eigen::Vector<float,3> &num, const Eigen::Vector<float,3> &den, float dt, Eigen::Vector<float,3> &numz, Eigen::Vector<float,3> &denz);
FilterError tf2ss(const Eigen::Vector<float,3> &num, const Eigen::Vector<float,3> &den, Eigen::Matrix<float,2,2> &A, Eigen::Matrix<float,2,1> &B, Eigen::Matrix<float,1,2> &C, Eigen::Matrix<float,1,1> &D);

}
