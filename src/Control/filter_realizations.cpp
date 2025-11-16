
#include "Control/control.hpp"
#include "Control/filter_realizations.hpp"

using namespace Control;

namespace Control {

FilterError tustin_1_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f;
    denz << 1.0f;

    if (num.size() < 1 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;

    float coeff_denom = 1.0f;
    coeff_denom = den(0) * K + den(1);

    // printf("tustin_1: den->k[0] = %0.3f, den->k[1] = %0.3f, K = %0.3f\n", den->k[0], den->k[1], K);

    if (coeff_denom < tol)
    {
        return FilterError::UNSTABLE;
    }

    // left pad or left truncate the numerator if num.size() != 2
    FiltVectorXf tmp_num = left_resize(num, 2);

    numz.resize(2);
    numz(0) = (tmp_num(0) * K + tmp_num(1)) / coeff_denom;
    numz(1) = (-tmp_num(0) * K + tmp_num(1)) / coeff_denom;

    denz.resize(2);
    denz(0) = 1.0f;
    denz(1) = (-den(0) * K + den(1)) / coeff_denom;

    return FilterError::NONE;
}

FilterError tustin_2_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f;
    denz << 1.0f;

    if (num.size() < 1 || den.size() < 3) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;
    float coeff_denom = den(0) * K * K + den(1) * K + den(2);

    if (coeff_denom < tol)
    {
        return FilterError::UNSTABLE;
    }

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, 3);
    numz.resize(3);
    numz(0) = (tmp_num(0)*K*K + tmp_num(1)*K + tmp_num(2)) / coeff_denom;
    numz(1) = (-2.0*tmp_num(0)*K*K       + 2.0*tmp_num(2)) / coeff_denom;
    numz(2) = (tmp_num(0)*K*K - tmp_num(1)*K + tmp_num(2)) / coeff_denom;

    denz.resize(3);
    denz(0) = 1.0f;
    denz(1) = (-2.0*den(0)*K*K       + 2.0*den(2)) / coeff_denom;
    denz(2) = (den(0)*K*K - den(1)*K + den(2)) / coeff_denom;

    return FilterError::NONE;
}

FilterError tustin_n_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;
    numz << 1.0f;
    denz << 1.0f;

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, n+1);

    Eigen::MatrixXf A(n,n);
    Eigen::MatrixXf B(n,1);
    Eigen::MatrixXf C(1,n);
    Eigen::MatrixXf D(1,1);

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    // this presumes the continuous transfer function is relative degree 1 or greater
    A << Eigen::MatrixXf::Zero(n-1,1),  Eigen::MatrixXf::Identity(n-1,n-1), den(Eigen::seq(Eigen::last,1,-1));
    B(n-1) = 1;
    C << tmp_num(Eigen::seq(Eigen::last,1,-1));
    // D = 0;

    // float K = 2.0f / dt;
    // float coeff_denom = den(0) * K * K + den(1) * K + den(2);

    // if (fabs(coeff_denom) < tol)
    // {
    //     return 3;
    // }

    // https://ocw.mit.edu/courses/6-245-multivariable-control-systems-spring-2004/e7aeed6b7a0d508ad3632c9a46b9a21d_lec11_6245_2004.pdf
    float w0 = 2.0f/dt;  // Nyquist frequency

    // (w0*I - A)^-1
    Eigen::MatrixXf invw0ImA = Eigen::Inverse(w0*Eigen::MatrixXf::Identity(n,n) - A);

    // using FPW's notation here
    Eigen::MatrixXf Phi = (w0*Eigen::MatrixXf::Identity(n,n) + A)*invw0ImA;
    Eigen::MatrixXf Gamma = sqrt(2*w0)*invw0ImA*B;
    Eigen::MatrixXf H = sqrt(2*w0)*C*invw0ImA;
    Eigen::MatrixXf J = D - C*invw0ImA*B;

    numz.resize(n+1);
    denz.resize(n+1);

    // TODO: convert from state space realization to discrete transfer function
    // this will require factorizing into a polynomial form
    // probably need to solve for the generalized eigenvalues/eigenvectors,
    // convert to Jordan form and then to a second order sections form

    return FilterError::NONE;
}

FilterError tf2ss( const FiltVectorXf &num, 
            const FiltVectorXf &den, 
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, NUM_STATES, NUM_STATES> &A, 
            Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES, 1> &B, 
            Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor, 1, NUM_STATES> &C, 
            Eigen::Matrix<float, 1, 1, 0, 1, 1> &D)
{

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION; // invalid vector dimension
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, n+1);

    // s->inf initial value subtracted out so that remaining num is relative degree one or greater
    float Ginf = tmp_num(0)/den(0);

    tmp_num -= Ginf*den;

    A.resize(n,n);
    B.resize(n,1);
    C.resize(1,n);
    D.resize(1,1);

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    A << Eigen::MatrixXf::Zero(n-1,1),  Eigen::MatrixXf::Identity(n-1,n-1), den(Eigen::seq(Eigen::last,1,-1));
    B(n-1) = 1;
    C << tmp_num(Eigen::seq(Eigen::last,1,-1));
    D << Ginf;

    return FilterError::NONE;
}

FilterError tf2ss(const Eigen::Vector3f &num, 
                  const Eigen::Vector3f &den,
                  Mat22 &A,
                  Mat21 &B,
                  Mat12 &C,
                  Mat11 &D)
{

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION; // invalid vector dimension
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, n+1);

    // s->inf initial value subtracted out so that remaining num is relative degree one or greater
    float Ginf = tmp_num(0)/den(0);

    tmp_num -= Ginf*den;

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    A << Eigen::MatrixXf::Zero(n-1,1),  Eigen::MatrixXf::Identity(n-1,n-1), den(Eigen::seq(Eigen::last,1,-1));
    B(n-1) = 1;
    C << tmp_num(Eigen::seq(Eigen::last,1,-1));
    D << Ginf;

    return FilterError::NONE;
}

FilterError tustin_2_tf(const Eigen::Vector3f &num, const Eigen::Vector3f &den, float dt, Eigen::Vector3f &numz, Eigen::Vector3f &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f, 0.0f, 0.0f;
    denz << 1.0f, 0.0f, 0.0f;

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;
    float coeff_denom = den(0) * K * K + den(1) * K + den(2);

    if (fabs(coeff_denom) < tol)
    {
        return FilterError::UNSTABLE;
    }

     // left pad or left truncate the numerator if num.size() != 3
    numz(0) = (num(0)*K*K + num(1)*K + num(2)) / coeff_denom;
    numz(1) = (-2.0*num(0)*K*K       + 2.0*num(2)) / coeff_denom;
    numz(2) = (num(0)*K*K - num(1)*K + num(2)) / coeff_denom;

    denz(0) = 1.0f;
    denz(1) = (-2.0*den(0)*K*K       + 2.0*den(2)) / coeff_denom;
    denz(2) = (den(0)*K*K - den(1)*K + den(2)) / coeff_denom;

    return FilterError::NONE;
}

}
