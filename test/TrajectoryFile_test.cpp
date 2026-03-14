#include "SimulationFrame.hpp"
#include "liteaerosim.pb.h"
#include <gtest/gtest.h>
#include <cmath>

// ---------------------------------------------------------------------------
// Step 12 — TrajectoryFile proto round-trip
// ---------------------------------------------------------------------------

// T1 — TrajectoryFile with 100 frames survives proto round-trip.
// Verifies: frame count, first timestamp, last timestamp.
TEST(TrajectoryFileTest, TrajectoryFile_ProtoRoundTrip_100Frames) {
    las_proto::TrajectoryFile tf;
    tf.set_schema_version(1);
    tf.set_world_origin_lat_rad(0.123456789012345);
    tf.set_world_origin_lon_rad(-0.987654321098765);
    tf.set_world_origin_h_m(150.5f);

    for (int i = 0; i < 100; ++i) {
        las_proto::TrajectoryFrame* frame = tf.add_frames();
        frame->set_timestamp_s(i * 0.01);
        frame->set_latitude_rad(0.1 + i * 1e-6);
        frame->set_longitude_rad(0.2 + i * 1e-6);
        frame->set_height_wgs84_m(100.f + static_cast<float>(i));
        frame->set_q_w(1.f);
        frame->set_q_x(0.f);
        frame->set_q_y(0.f);
        frame->set_q_z(0.f);
    }

    // Serialize → deserialize.
    const std::string bytes = tf.SerializeAsString();
    las_proto::TrajectoryFile tf2;
    ASSERT_TRUE(tf2.ParseFromString(bytes));

    EXPECT_EQ(tf2.schema_version(), 1);
    EXPECT_NEAR(tf2.world_origin_lat_rad(), 0.123456789012345, 1e-12);
    EXPECT_NEAR(tf2.world_origin_lon_rad(), -0.987654321098765, 1e-12);
    EXPECT_NEAR(tf2.world_origin_h_m(), 150.5f, 1e-5f);

    ASSERT_EQ(tf2.frames_size(), 100);
    EXPECT_NEAR(tf2.frames(0).timestamp_s(),   0.0,  1e-12);
    EXPECT_NEAR(tf2.frames(99).timestamp_s(), 99 * 0.01, 1e-12);
}

// T2 — TrajectoryFrame: all fields survive the round-trip within tolerance.
// Doubles: ± 1e-9; floats: ± 1e-6.
TEST(TrajectoryFileTest, TrajectoryFrame_AllFieldsRoundTrip) {
    las_proto::TrajectoryFrame frame;
    frame.set_timestamp_s(12345.6789012345);
    frame.set_latitude_rad(0.523598775598299);   // 30 deg in rad
    frame.set_longitude_rad(1.570796326794897);  // 90 deg in rad
    frame.set_height_wgs84_m(3500.25f);
    frame.set_q_w(0.7071068f);
    frame.set_q_x(0.0f);
    frame.set_q_y(0.7071068f);
    frame.set_q_z(0.0f);

    const std::string bytes = frame.SerializeAsString();
    las_proto::TrajectoryFrame frame2;
    ASSERT_TRUE(frame2.ParseFromString(bytes));

    EXPECT_NEAR(frame2.timestamp_s(),    12345.6789012345,     1e-9);
    EXPECT_NEAR(frame2.latitude_rad(),   0.523598775598299,    1e-9);
    EXPECT_NEAR(frame2.longitude_rad(),  1.570796326794897,    1e-9);
    EXPECT_NEAR(frame2.height_wgs84_m(), 3500.25f,             1e-6f);
    EXPECT_NEAR(frame2.q_w(),            0.7071068f,           1e-6f);
    EXPECT_NEAR(frame2.q_x(),            0.0f,                 1e-6f);
    EXPECT_NEAR(frame2.q_y(),            0.7071068f,           1e-6f);
    EXPECT_NEAR(frame2.q_z(),            0.0f,                 1e-6f);
}
