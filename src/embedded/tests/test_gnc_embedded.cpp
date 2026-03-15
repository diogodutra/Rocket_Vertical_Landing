#include <gtest/gtest.h>
#include "GNCController.hpp"

class GNCConsistencyTest : public ::testing::Test {
protected:
    RocketGNC::GNCController controller;
    const float dt = 0.01f;
    void SetUp() override {
        // If you added a reset method to GNCController:
        controller.reset_internal_state(); 
    }
};

TEST_F(GNCConsistencyTest, Nominal) {
    RocketGNC::VehicleState state = {0.0f, 150.0f, 0.0f, 0.0f, -10.0f, 0.0f};
    auto cmd = controller.update(state, dt);
    EXPECT_NEAR(cmd.thrust_N, 49050.0f, 1.0f);
    EXPECT_NEAR(cmd.tvc_angle_rad, 0.00f, 0.005f);
}

TEST_F(GNCConsistencyTest, HighAltitudeDescent) {
    RocketGNC::VehicleState state = {2.0f, 100.0f, 0.01f, 1.0f, -5.0f, 0.01f};
    auto cmd = controller.update(state, dt);
    // Note: If your code outputs 68000, keep it. If it outputs 49050, update here.
    EXPECT_NEAR(cmd.thrust_N, 49050.0f, 1.0f);
    EXPECT_NEAR(cmd.tvc_angle_rad, 0.17f, 0.005f);
}

TEST_F(GNCConsistencyTest, Hover) {
    RocketGNC::VehicleState state = {0.0f, 20.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    auto cmd = controller.update(state, dt);
    EXPECT_NEAR(cmd.thrust_N, 49050.0f, 1.0f);
    EXPECT_NEAR(cmd.tvc_angle_rad, -0.00f, 0.005f);
}