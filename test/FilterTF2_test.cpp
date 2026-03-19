#define _USE_MATH_DEFINES
#include "control/FilterTF2.hpp"
#include "control/filter_realizations.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace liteaerosim::control;
using namespace liteaerosim;

TEST(FilterTF2Test, Instantiation00) {

    FilterTF2 G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    EXPECT_EQ(G.order(), 0);

}

TEST(FilterTF2Test, Update00) {

    FilterTF2 G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    EXPECT_EQ(G.order(), 0);

    EXPECT_EQ(G.step(1.0f), 1.0f);
}

TEST(FilterTF2Test, FirstOrderLP00) {

    FilterTF2 G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float tau = 10;

    G.setLowPassFirstIIR(dt, tau);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 1);

    EXPECT_NEAR(G.step(1.0f), 0.00497512437810943f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.014875869409173084f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.024678099564305757f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.03438279509102915f, 1e-3);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 1);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    for (int k = 0; k < 100; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 0.6302749995213918f, 1e-3);
    
}

TEST(FilterTF2Test, SecondOrderLP00) {

    FilterTF2 G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float wn_rps = 2.0;
    float zeta = 1.0/sqrt(2.0);
    float tau_zero = 0.0f;

    Vec3 num_s(0,0,wn_rps*wn_rps);
    Vec3 den_s(1.0,2.0*zeta*wn_rps,wn_rps*wn_rps);

    Vec3 num_z0;
    Vec3 den_z0;

    tustin_2_tf(num_s, den_s, dt, 0.0f, num_z0, den_z0);

    G.setLowPassSecondIIR(dt, wn_rps, zeta, tau_zero);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    Vec3 num_z(G.num());
    Vec3 den_z(G.den());

    float Ginf = num_z(0)/den_z(0);

    Vec3 tmp_num = num_z - Ginf*den_z;

    // EXPECT_EQ(Phi(0,0),0.0f);
    // EXPECT_EQ(Phi(0,1),1.0f);
    // EXPECT_NEAR(Phi(1,0),-den_z(2), 1e-8);
    // EXPECT_NEAR(Phi(1,1),-den_z(1), 1e-8);

    // EXPECT_EQ(Gamma(0,0),0.0f);
    // EXPECT_EQ(Gamma(1,0),1.0f);

    // EXPECT_NEAR(H(0,0),tmp_num(2), 1e-8);
    // EXPECT_NEAR(H(0,1),tmp_num(1), 1e-8);

    // EXPECT_NEAR(J(0,0),0.008684917945832371f, 1e-8);

    EXPECT_NEAR(G.step(1.0f), 0.008684917945832371f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.04098945818321358f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.09867421021567828f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.1735006625919544f, 1e-3);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    for (int k = 0; k < 20; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 1.035730817247945f, 1e-3);

}

TEST(FilterTF2Test, SchemaVersionAndTypeName) {
    FilterTF2 G;
    nlohmann::json snap = G.serializeJson();
    EXPECT_EQ(snap["schema_version"].get<int>(), 1);
    EXPECT_EQ(snap["type"].get<std::string>(), "FilterTF2");
}

TEST(FilterTF2Test, NviInAndOutUpdated) {
    FilterTF2 G;
    G.setLowPassFirstIIR(0.1f, 10.0f);
    SisoElement* base = &G;
    base->step(1.0f);
    EXPECT_EQ(base->in(), 1.0f);
    EXPECT_NE(base->out(), 0.0f);
}

TEST(FilterTF2Test, SerializeNonEmpty) {
    FilterTF2 G;
    G.setLowPassFirstIIR(0.1f, 10.0f);
    G.step(1.0f);
    G.step(1.0f);
    nlohmann::json snap = G.serializeJson();
    EXPECT_EQ(snap["schema_version"].get<int>(), 1);
    EXPECT_EQ(snap["type"].get<std::string>(), "FilterTF2");
    EXPECT_TRUE(snap.contains("order"));
    EXPECT_TRUE(snap.contains("state"));
}

TEST(FilterTF2Test, JsonRoundTrip) {
    FilterTF2 G;
    G.setLowPassFirstIIR(0.1f, 10.0f);
    G.step(1.0f);
    G.step(1.0f);
    nlohmann::json snap = G.serializeJson();
    FilterTF2 G2;
    G2.setLowPassFirstIIR(0.1f, 10.0f);
    G2.deserializeJson(snap);
    EXPECT_FLOAT_EQ(G2.step(1.0f), G.step(1.0f));
}
