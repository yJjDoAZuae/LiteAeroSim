
#include "Control/control.hpp"
#include "Control/filter_realizations.hpp"

using namespace Control;

namespace Control {

// https://en.wikipedia.org/wiki/Butterworth_filter#Normalized_Butterworth_polynomials
FilterError butter(char order, float dt, float wn_rps, FiltVectorXf& num_s, FiltVectorXf& den_s)
{
    if (order < 1 || order > 10 || order > NUM_STATES) {
        return FilterError::INVALID_DIMENSION;
    }

    num_s.resize(1);
    den_s.resize(order+1);

    num_s << 1;
    float a = 1/wn_rps;
    switch (order) {
        case 0:
            // DC pass through
            den_s << 1;
            break;

        case 1:
            den_s << 1, 1;
            break;
        case 2:
            den_s << 1, 2.0/sqrt(2.0), 1;
            break;
        case 3:
            den_s << 1, 2, 2, 1;
            break;
        case 4:
            den_s << 1, 2.6131, 3.4142, 2.6131, 1;
            break;
        case 5:
            den_s << 1, 3.2361, 5.2361, 5.2361, 3.2361, 1;
            break;
        case 6:
            den_s << 1, 3.8637, 7.4641, 9.1416, 7.4641, 3.8637, 1;
            break;
        case 7:
            den_s << 1, 4.4940, 10.0978, 14.5918, 14.5918, 10.0978, 4.4940, 1;
            break;
        case 8:
            den_s << 1, 5.1258, 13.1371, 21.8462, 25.6884, 21.8462, 13.1371, 5.1258, 1;
            break;
        case 9:
            den_s << 1, 5.7588, 16.5817, 31.1634, 41.9864, 41.9864, 31.1634, 16.5817, 5.7588, 1;
            break;
        case 10:
            den_s << 1, 6.3925, 20.4317, 42.8021, 64.8824, 74.2334, 64.8824, 42.8021, 20.4317, 6.3925, 1;
            break;
    }

    // update the coefficients for wn_rps
    for (int k = 0; k < den_s.size(); k++) {
        den_s(k) *= 1/pow(wn_rps, order-k);
    }

    return FilterError::NONE;
}

FilterError tustin_1_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz.resize(1);
    denz.resize(1);
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

    numz.resize(1);
    denz.resize(1);
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
    numz.resize(1);
    denz.resize(1);
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

FilterError tustin_n_ss(const MatNN &A,
                        const MatN1 &B,
                        const Mat1N &C,
                        const Mat11 &D,
                        float dt,
                        MatNN &Phi,
                        MatN1 &Gamma,
                        Mat1N &H,
                        Mat11 &J )
{
    const float tol = 1.0e-6;

    if (A.size() == 0 || B.size() == 0 || C.size() == 0 || D.size() == 0) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    char n = A.rows();

    Phi.resize(n,n);
    Gamma.resize(n,1);
    H.resize(1,n);
    J.resize(1,1);

    Phi.setZero();
    Gamma.setZero();
    H.setZero();
    J.setZero();

    // https://ocw.mit.edu/courses/6-245-multivariable-control-systems-spring-2004/e7aeed6b7a0d508ad3632c9a46b9a21d_lec11_6245_2004.pdf
    float w0 = 2.0f/dt;  // Nyquist frequency

    // (I - A/w0)^-1 // from FPW p. 200
    Eigen::MatrixXf invw0ImA = Eigen::Inverse(Eigen::MatrixXf::Identity(n,n) - A/w0);

    // using FPW's notation here
    Phi = (Eigen::MatrixXf::Identity(n,n) + A/w0)*invw0ImA;
    Gamma =invw0ImA*B*sqrt(dt);
    H = sqrt(dt)*C*invw0ImA;
    J = D + C*invw0ImA*B/w0;

    return FilterError::NONE;
}

FilterError tf2ss( const FiltVectorXf &num, 
            const FiltVectorXf &den, 
            MatNN &A, 
            MatN1 &B, 
            Mat1N &C, 
            Mat11 &D)
{

    static const float tol = 1e-9;

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION; // invalid vector dimension
    }

    float d0 = den(0);
    if (fabs(d0) < tol) {
        return FilterError::INVALID_POLYNOMIAL;
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() < den.size()
     // normalize both num and den so that den(0) = 1
    FiltVectorXf tmp_num = left_resize(num/d0, n+1);
    FiltVectorXf tmp_den = den/d0;

    // s->inf initial value subtracted out so that remaining num is relative degree one or greater
    float Ginf = tmp_num(0);
    tmp_num -= Ginf * tmp_den; // tmp_num(0) = 0 after this

    A.resize(n, n);
    B.resize(n, 1);
    C.resize(1, n);
    D.resize(1, 1);

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    if (n > 1) {

        A.block(0,0,n-1,1).setZero();
        A.block(0,1,n-1,n-1) << MatNN::Identity(n-1,n-1);
        A.block(n-1,0,1,n) = Mat1N(-tmp_den(Eigen::seqN(n,n,Eigen::fix<-1>)));

        B(n - 1) = 1;
        C = Mat1N(tmp_num(Eigen::seqN(n, n, Eigen::fix<-1>)));
        D << Ginf;
    } else {
        A << -tmp_den(Eigen::last);
        B << 1;
        C << tmp_num(Eigen::last);
        D << Ginf;
    }

    return FilterError::NONE;
}

FilterError tf2ss(const Eigen::Vector3f &num, 
                  const Eigen::Vector3f &den,
                  Mat22 &A,
                  Mat21 &B,
                  Mat12 &C,
                  Mat11 &D)
{

    // s->inf initial value subtracted out so that remaining num is relative degree one or greater
    float Ginf = num(0)/den(0);

    Eigen::Vector3f tmp_num = num - Ginf*den;

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    A << 0.0f, 1.0f, -den(2), -den(1);
    B(1) = 1.0f;
    C << tmp_num(2), tmp_num(1);
    D << Ginf;

    return FilterError::NONE;
}

FilterError tustin_1_tf(const Eigen::Vector3f &num, const Eigen::Vector3f &den, float dt, Eigen::Vector3f &numz, Eigen::Vector3f &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f, 0.0f, 0.0f;
    denz << 1.0f, 0.0f, 0.0f;

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;

    float coeff_denom = den(1) * K + den(2);

    // printf("tustin_1: den->k[0] = %0.3f, den->k[1] = %0.3f, K = %0.3f\n", den->k[0], den->k[1], K);

    if (fabs(coeff_denom) < tol)
    {
        return FilterError::UNSTABLE;
    }

    numz(0) = (num(1) * K + num(2)) / coeff_denom;
    numz(1) = (-num(1) * K + num(2)) / coeff_denom;

    denz(0) = 1.0f;
    denz(1) = (-den(1) * K + den(2)) / coeff_denom;

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

    numz(0) = (num(0)*K*K + num(1)*K + num(2)) / coeff_denom;
    numz(1) = (-2.0*num(0)*K*K       + 2.0*num(2)) / coeff_denom;
    numz(2) = (num(0)*K*K - num(1)*K + num(2)) / coeff_denom;

    denz(0) = 1.0f;
    denz(1) = (-2.0*den(0)*K*K       + 2.0*den(2)) / coeff_denom;
    denz(2) = (den(0)*K*K - den(1)*K + den(2)) / coeff_denom;

    return FilterError::NONE;
}


FilterError tustin_2_ss(const Mat22 &A,
                        const Mat21 &B,
                        const Mat12 &C,
                        const Mat11 &D,
                        float dt,
                        Mat22 &Phi,
                        Mat21 &Gamma,
                        Mat12 &H,
                        Mat11 &J )
{
    const float tol = 1.0e-6;

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    Phi.setZero();
    Gamma.setZero();
    H.setZero();
    J.setZero();

    // https://ocw.mit.edu/courses/6-245-multivariable-control-systems-spring-2004/e7aeed6b7a0d508ad3632c9a46b9a21d_lec11_6245_2004.pdf
    float w0 = 2.0f/dt;  // Nyquist frequency

    // (I - A/w0)^-1 // from FPW p. 200
    Mat22 invw0ImA = Eigen::Inverse(Mat22::Identity(2,2) - A/w0);

    // using FPW's notation here
    Phi = (Mat22::Identity(2,2) + A/w0)*invw0ImA;
    Gamma =invw0ImA*B*sqrt(dt);
    H = sqrt(dt)*C*invw0ImA;
    J = D + C*invw0ImA*B/w0;

    return FilterError::NONE;
}


}
