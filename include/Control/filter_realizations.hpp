#pragma once

#include <Eigen/Dense>
#include "Control/control.hpp"

namespace Control {

FilterError tustin_1_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FilterError tustin_2_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FilterError tustin_n_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);

FilterError tf2ss(const FiltVectorXf &num, 
                  const FiltVectorXf &den, 
                  MatNN &A,
                  MatN1 &B,
                  Mat1N &C,
                  Mat11 &D);

FilterError tustin_1_tf(const Eigen::Vector3f &num, 
                        const Eigen::Vector3f &den, 
                        float dt, 
                        Eigen::Vector3f &numz, 
                        Eigen::Vector3f &denz);

FilterError tustin_2_tf(const Eigen::Vector3f &num, 
                        const Eigen::Vector3f &den, 
                        float dt, 
                        Eigen::Vector3f &numz, 
                        Eigen::Vector3f &denz);

FilterError tf2ss(const Eigen::Vector3f &num, 
                  const Eigen::Vector3f &den, 
                  Mat22 &A, 
                  Mat21 &B, 
                  Mat12 &C, 
                  Mat11 &D);

}
