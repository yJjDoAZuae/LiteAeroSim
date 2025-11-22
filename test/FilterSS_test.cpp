#define _USE_MATH_DEFINES
#include "control/FilterSS.hpp"
#include "control/filter_realizations.hpp"
#include <gtest/gtest.h>

using namespace Control;

TEST(FilterSSTest, Instantiation00) {

    FilterSS G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    //EXPECT_EQ(G.order(), 0);

}

TEST(FilterSSTest, Order00) {

    FilterSS G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    EXPECT_EQ(G.order(), 0);

}

TEST(FilterSSTest, Update00) {

    FilterSS G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    EXPECT_EQ(G.order(), 0);

    EXPECT_EQ(G.step(1.0f), 1.0f);
}

TEST(FilterSSTest, SecondOrderLP00) {

    FilterSS G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float wn_rps = 2.0;
    float zeta = 1.0/sqrt(2.0);
    float tau_zero = 0.0f;

    FiltVectorXf num_s;
    FiltVectorXf den_s;
    num_s.resize(1);
    den_s.resize(3);
    num_s << wn_rps*wn_rps;
    den_s << 1.0,2.0*zeta*wn_rps,wn_rps*wn_rps;

    FiltVectorXf num_z;
    FiltVectorXf den_z;

    MatNN A;
    MatN1 B;
    Mat1N C;
    Mat11 D;

    tf2ss(num_s, den_s, A, B, C, D);

    MatNN Phi0;
    MatN1 Gamma0;
    Mat1N H0;
    Mat11 J0;

    tustin_n_ss(A, B, C, D, dt, wn_rps, Phi0, Gamma0, H0, J0);

    G.setButterworthIIR(2, dt, wn_rps);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    MatNN Phi(G.Phi());
    MatN1 Gamma(G.Gamma());
    Mat1N H(G.H());
    Mat11 J(G.J());

    // float Ginf = num_z(0)/den_z(0);

    // Eigen::Vector3f tmp_num = num_z - Ginf*den_z;

    EXPECT_EQ(Phi.rows(),2);
    EXPECT_EQ(Phi.cols(),2);

    EXPECT_EQ(Gamma.rows(),2);
    EXPECT_EQ(Gamma.cols(),1);

    EXPECT_EQ(H.rows(),1);
    EXPECT_EQ(H.cols(),2);

    EXPECT_EQ(J.rows(),1);
    EXPECT_EQ(J.cols(),1);

    EXPECT_NEAR(Phi(0,0),Phi0(0,0), 1e-6);
    EXPECT_NEAR(Phi(0,1),Phi0(0,1), 1e-6);
    EXPECT_NEAR(Phi(1,0),Phi0(1,0), 1e-6);
    EXPECT_NEAR(Phi(1,1),Phi0(1,1), 1e-6);

    EXPECT_NEAR(Gamma(0,0),Gamma0(0,0), 1e-6);
    EXPECT_NEAR(Gamma(1,0),Gamma0(1,0), 1e-6);

    EXPECT_NEAR(H(0,0),H0(0,0), 1e-6);
    EXPECT_NEAR(H(0,1),H0(0,1), 1e-6);

    EXPECT_NEAR(J(0,0),0.008739046114517035, 1e-6);

    EXPECT_NEAR(G.step(1.0f), 0.008739046114517035, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.041236855975003976, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.09924343173467982, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.1744469218931253, 1e-6);

    G.resetInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    for (int k = 0; k < 20; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 1.0361699725173787, 1e-6);
    
}
