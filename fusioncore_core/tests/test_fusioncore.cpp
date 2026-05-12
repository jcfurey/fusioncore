#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/gnss.hpp"

using namespace fusioncore;

// ─── Test 1: Cannot update before init ───────────────────────────────────────

TEST(FusionCoreTest, ThrowsIfNotInitialized) {
  FusionCore fc;
  EXPECT_THROW(fc.update_imu(0.0, 0,0,0,0,0,0), std::runtime_error);
  EXPECT_THROW(fc.update_encoder(0.0, 0,0,0),    std::runtime_error);
}

// ─── Test 2: Initializes cleanly ─────────────────────────────────────────────

TEST(FusionCoreTest, InitializesCleanly) {
  FusionCore fc;

  State initial;
  initial.x     = StateVector::Zero();
  initial.x[X]  = 1.0;
  initial.x[Y]  = 2.0;
  initial.P     = StateMatrix::Identity() * 0.1;

  fc.init(initial, 0.0);

  EXPECT_TRUE(fc.is_initialized());
  EXPECT_DOUBLE_EQ(fc.get_state().x[X], 1.0);
  EXPECT_DOUBLE_EQ(fc.get_state().x[Y], 2.0);
}

// ─── Test 3: Status reflects sensor health ───────────────────────────────────

TEST(FusionCoreTest, StatusReflectsSensorHealth) {
  FusionCore fc;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Before any sensor data
  auto status = fc.get_status();
  EXPECT_EQ(status.imu_health,     SensorHealth::NOT_INIT);
  EXPECT_EQ(status.encoder_health, SensorHealth::NOT_INIT);

  // Send gravity on az: the measurement model now predicts ~9.81 for a flat
  // stationary robot, so the innovation is near zero and the update is accepted.
  fc.update_imu(0.01, 0,0,0, 0,0,9.81);
  status = fc.get_status();
  EXPECT_EQ(status.imu_health, SensorHealth::OK);
  EXPECT_EQ(status.update_count, 1);
}

// ─── Test 4: Position uncertainty grows without updates ──────────────────────

TEST(FusionCoreTest, UncertaintyGrowsWithoutUpdates) {
  FusionCore fc;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.01;
  fc.init(initial, 0.0);

  double initial_uncertainty = fc.get_status().position_uncertainty;

  // Feed only IMU: no encoder, no position corrections
  for (int i = 1; i <= 100; ++i) {
    fc.update_imu(i * 0.01, 0,0,0,0,0,9.8);
  }

  double final_uncertainty = fc.get_status().position_uncertainty;
  EXPECT_GT(final_uncertainty, initial_uncertainty);
}

// ─── Test 5: Full end-to-end: robot drives forward 1 meter ─────────────────

TEST(FusionCoreTest, RobotDrivesForwardOneMeter) {
  FusionCoreConfig config;
  config.ukf.q_position    = 1e-6;
  config.ukf.q_velocity    = 1e-6;
  config.ukf.q_orientation = 1e-6;
  config.ukf.q_angular_vel = 1e-6;
  config.ukf.q_acceleration= 1e-6;
  config.ukf.q_gyro_bias   = 1e-6;
  config.ukf.q_accel_bias  = 1e-6;

  FusionCore fc(config);

  // Large uncertainty on position/velocity: we don't know where we are.
  // Small uncertainty on orientation: robot starts at a known heading (yaw=0).
  // High quaternion uncertainty would spread sigma points broadly and
  // collapse the cos(yaw) average toward zero during position integration.
  // Keep orientation uncertainty tight; large uncertainty is on position/velocity.
  State initial;
  initial.P = StateMatrix::Identity() * 1.0;
  initial.P(QW,QW) = 0.01;
  initial.P(QX,QX) = 0.01;
  initial.P(QY,QY) = 0.01;
  initial.P(QZ,QZ) = 0.01;
  fc.init(initial, 0.0);

  // Robot drives forward at 1 m/s for 1 second
  // IMU at 100Hz, encoder at 50Hz
  for (int i = 1; i <= 100; ++i) {
    double t = i * 0.01;

    // IMU: moving forward at constant velocity, flat robot: send gravity on az.
    fc.update_imu(t, 0,0,0, 0,0,9.81);

    // Encoder at 50Hz
    if (i % 2 == 0) {
      fc.update_encoder(t, 1.0, 0.0, 0.0);
    }
  }

  // Should be approximately 1 meter forward in X
  // Tolerance is 0.5m: IMU sends zero acceleration while encoder sends 1m/s velocity.
  // The filter correctly reconciles conflicting sensors: position converges toward 1m
  // but not exactly due to the physically inconsistent input combination.
  EXPECT_NEAR(fc.get_state().x[X], 1.0, 0.5);
  EXPECT_NEAR(fc.get_state().x[Y], 0.0, 0.1);
}

// ─── Test 6: Reset clears state ──────────────────────────────────────────────

TEST(FusionCoreTest, ResetClearsState) {
  FusionCore fc;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  fc.update_imu(0.01, 0,0,1, 0,0,9.8);
  EXPECT_TRUE(fc.is_initialized());

  fc.reset();
  EXPECT_FALSE(fc.is_initialized());
}

// ─── Test 7: 6-axis IMU: yaw blocked, roll/pitch still fused ────────────────
// When imu_has_magnetometer=false, update_imu_orientation calls a 2D
// IMU_RP_DIM update path (sensors::imu_rp_measurement_function) that fuses
// only roll and pitch — yaw is never presented to the UKF. A wildly wrong
// yaw measurement therefore cannot move the filter's heading; roll and
// pitch must still converge normally. (Earlier revisions implemented this
// as a 3D update with R(yaw) inflated to 1e6, hence "blocked yaw" wording
// elsewhere; the current implementation is cleaner.)

TEST(FusionCoreTest, SixAxisIMUYawBlockedRollPitchFused) {
  FusionCoreConfig config;
  config.imu_has_magnetometer = false;
  config.adaptive_imu = false;  // keep R stable: we're testing the fix, not adaption

  FusionCore fc(config);

  State initial;
  // roll = 0.3 rad → quaternion [cos(0.15), sin(0.15), 0, 0]; yaw stays 0
  initial.x[QW] = std::cos(0.15);
  initial.x[QX] = std::sin(0.15);
  initial.x[QY] = 0.0;
  initial.x[QZ] = 0.0;
  initial.P         = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Feed 200 orientation updates: correct roll=0, but yaw=π (wildly wrong).
  // The 2D update never sees yaw, so the wildly wrong value has no effect.
  for (int i = 1; i <= 200; ++i) {
    fc.update_imu_orientation(i * 0.01, 0.0, 0.0, M_PI, nullptr);
  }

  // Yaw must not have moved: the fix blocks it
  EXPECT_NEAR(fc.get_state().yaw(), 0.0, 0.01);

  // Roll must have converged: R(0,0) is normal so gain is high
  EXPECT_NEAR(fc.get_state().roll(), 0.0, 0.05);
}

// ─── Test 8: 9-axis IMU: yaw IS fused normally ──────────────────────────────
// When imu_has_magnetometer=true, the fix is skipped entirely.
// The yaw measurement must pull the filter heading toward the target.

TEST(FusionCoreTest, NineAxisIMUYawFusedNormally) {
  FusionCoreConfig config;
  config.imu_has_magnetometer = true;
  config.adaptive_imu = false;

  FusionCore fc(config);

  State initial;
  // State() default-constructs with QW=1 (identity quaternion) = yaw 0
  initial.P       = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Feed 200 orientation updates with yaw=0.5 rad.
  // Small enough that the first update passes the Mahalanobis gate
  // (d² ≈ 0.25/0.1025 ≈ 2.4, well below the 15.09 threshold).
  for (int i = 1; i <= 200; ++i) {
    fc.update_imu_orientation(i * 0.01, 0.0, 0.0, 0.5, nullptr);
  }

  // Yaw must have converged toward 0.5
  EXPECT_GT(fc.get_state().yaw(), 0.3);
}

// ─── Test 9: GNSS coast mode triggers after N consecutive rejections ────────
// Verifies the gnss_in_coast / gnss_consecutive_rejects fields added to
// FusionCoreStatus (commit 9c55cfa). Drives the filter into a tight position
// state, then feeds GPS fixes far enough away that the chi² gate rejects
// every one. After config.gnss_coast_n consecutive rejects, coast mode must
// activate and the counter must reflect the rejection streak.

TEST(FusionCoreTest, GnssCoastModeActivatesAfterConsecutiveRejections) {
  FusionCoreConfig config;
  config.adaptive_imu  = false;
  config.adaptive_gnss = false;
  config.gnss_coast_n  = 3;  // make the test cheap

  FusionCore fc(config);

  // Tight initial uncertainty so a far-off fix has huge Mahalanobis distance.
  State initial;
  initial.P = StateMatrix::Identity() * 1e-4;
  fc.init(initial, 0.0);

  // Feed a few IMU samples so last_timestamp_ advances and the predict step
  // doesn't blow up the position covariance back out to "any fix is fine".
  for (int i = 1; i <= 5; ++i) {
    fc.update_imu(i * 0.01, 0,0,0, 0,0,9.81);
  }

  // Build a fix that passes is_valid() but is wildly off in position.
  // Default base_noise_xy = 1.0, so HDOP*base = 1m sigma; pos at (1000,0,0)
  // is ~1000σ → d² ~= 1e6, well past outlier_threshold_gnss = 16.27.
  sensors::GnssFix fix;
  fix.x = 1000.0;
  fix.y = 0.0;
  fix.z = 0.0;
  fix.hdop = 1.0;
  fix.vdop = 1.0;
  fix.satellites = 4;
  fix.fix_type = sensors::GnssFixType::GPS_FIX;

  // Send coast_n + 1 fixes; coast must engage by the coast_n-th rejection.
  for (int i = 1; i <= config.gnss_coast_n + 1; ++i) {
    fc.update_gnss(0.05 + i * 0.01, fix);
  }

  auto status = fc.get_status();
  EXPECT_TRUE(status.gnss_in_coast);
  EXPECT_GE(status.gnss_consecutive_rejects, config.gnss_coast_n);
  EXPECT_GE(status.gnss_outliers, config.gnss_coast_n);

  // A fix back near the predicted position must clear coast and reset the counter.
  fix.x = 0.0;
  fc.update_gnss(0.5, fix);
  status = fc.get_status();
  EXPECT_FALSE(status.gnss_in_coast);
  EXPECT_EQ(status.gnss_consecutive_rejects, 0);
}

// ─── Test 10: Per-sensor age fields advance with last_timestamp_ ────────────
// Verifies imu_age, encoder_age, gnss_age (commit 9c55cfa) report the gap
// between the most recent measurement of each kind and the filter's clock.

TEST(FusionCoreTest, PerSensorAgesReportTimeSinceLastMeasurement) {
  FusionCore fc;
  State initial;
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Before any measurement: ages should be -1 (never received).
  auto status = fc.get_status();
  EXPECT_LT(status.imu_age,     0.0);
  EXPECT_LT(status.encoder_age, 0.0);
  EXPECT_LT(status.gnss_age,    0.0);

  fc.update_imu(1.0, 0,0,0, 0,0,9.81);
  fc.update_encoder(2.0, 0,0,0);

  // Drive last_timestamp_ to 3.0 with another IMU update; then encoder is
  // 1s old, IMU is 0s old.
  fc.update_imu(3.0, 0,0,0, 0,0,9.81);

  status = fc.get_status();
  EXPECT_NEAR(status.imu_age,     0.0, 1e-9);
  EXPECT_NEAR(status.encoder_age, 1.0, 1e-9);
  EXPECT_LT(status.gnss_age, 0.0);  // GNSS still never received
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
