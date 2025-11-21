#pragma once

#include <Eigen/Dense>
#include "control/control.hpp"

namespace Control {

FilterError butter(char order, float dt, float wn_rps, FiltVectorXf& num_s, FiltVectorXf& den_s);
FilterError tustin_1_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FilterError tustin_2_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FilterError tustin_n_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);

FilterError tustin_n_ss(const MatNN &A,
                        const MatN1 &B,
                        const Mat1N &C,
                        const Mat11 &D,
                        float dt,
                        MatNN &Phi,
                        MatN1 &Gamma,
                        Mat1N &H,
                        Mat11 &J );

FilterError tf2ss(const FiltVectorXf &num, 
                  const FiltVectorXf &den, 
                  MatNN &A,
                  MatN1 &B,
                  Mat1N &C,
                  Mat11 &D);

FilterError tustin_1_tf(const Vec3 &num, 
                        const Vec3 &den, 
                        float dt, 
                        Vec3 &numz, 
                        Vec3 &denz);

FilterError tustin_2_tf(const Vec3 &num, 
                        const Vec3 &den, 
                        float dt, 
                        Vec3 &numz, 
                        Vec3 &denz);

FilterError tustin_2_ss(const Mat22 &A,
                        const Mat21 &B,
                        const Mat12 &C,
                        const Mat11 &D,
                        float dt,
                        Mat22 &Phi,
                        Mat21 &Gamma,
                        Mat12 &H,
                        Mat11 &J );

FilterError tf2ss(const Vec3 &num, 
                  const Vec3 &den, 
                  Mat22 &A, 
                  Mat21 &B, 
                  Mat12 &C, 
                  Mat11 &D);

}
