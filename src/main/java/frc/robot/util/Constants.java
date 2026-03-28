// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

/**
 * The Constants class provides a single place for robot-wide numerical and boolean constants.
 * This class should not contain logic. All values should be declared as public static fields.
 *
 * It is recommended to statically import this class (or one of its inner classes) where needed.
 */
public final class Constants {
  private Constants() {} // Prevent instantiation

  /* ================= Robot Physical Properties ================= */

  public static final class RobotProperties {
    private RobotProperties() {} // Prevent instantiation

    /** Robot mass in kilograms (measured weight minus bumpers). */
    public static final double ROBOT_MASS = Units.lbsToKilograms(148 - 20.3); // lbs to kg

    /** Center of mass used for swerve dynamics calculations. */
    public static final Matter CHASSIS = new Matter(
        new Translation3d(0, 0, Units.inchesToMeters(8)),
        ROBOT_MASS);

    /** Control loop period in seconds (20ms DS + ~110ms controller latency). */
    public static final double LOOP_TIME = 0.13;

    /** Maximum linear speed of the robot in meters per second. */
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    /** Distance from the camera to the front bumper in meters. */
    public static final double CAM_TO_SHOOTER_DISTANCE = Units.feetToMeters(1.5);
  }

  /* ================= Shooter ================= */

  public static final class ShooterConstants {
    private ShooterConstants() {} // Prevent instantiation

    /** CAN ID for the primary shooter motor. */
    public static final int SHOOTER_MOTOR_ID1 = 39;

    /** CAN ID for the follower shooter motor. */
    public static final int SHOOTER_MOTOR_ID2 = 23;

    /** Gear ratio from motor to shooter wheel (motor rotations per wheel rotation). */
    public static final double GEAR_RATIO = 1.0;

    /** Minimum valid distance to the target (in meters) used to clamp Limelight-derived values. */
    public static final double MINIMUM_DISTANCE = Units.inchesToMeters(17); // 17 inches to meters

    /** Maximum valid distance to the target (in meters) used to clamp Limelight-derived values. */
    public static final double MAXIMUM_DISTANCE = Units.inchesToMeters(250); // 250 inches to meters

    /** Gravity constant (m/s^2) used for projectile motion calculations. */
    public static final double GRAVITY_CONSTANT = 9.81;

    /** Physical shooter wheel radius in meters (used as the baseline before efficiency scaling). */
    public static final double SHOOTER_WHEEL_RADIUS = Units.inchesToMeters(2.25); // 2.25 inches to meters

    /** 
     * Key for tuning the effective radius efficiency on the dashboard. This allows live adjustments to
     * account for real-world losses without changing the physical constants. The value should be in the 
     * range (0, 1], where 1 means no losses and values closer to 0 represent more significant losses. 
     * Tuning this value effectively scales the computed wheel radius used in velocity calculations, 
     * allowing you to empirically match the theoretical projectile motion to actual shot performance.
     *
     * TUNING GUIDE:
     * 1) Set up the robot at a known, repeatable distance inside MINIMUM/MAXIMUM range.
     * 2) Command a shot and observe whether shots fall short or overshoot.
     * 3) Increase the value to raise the computed RPS (shots go farther).
     * 4) Decrease the value to lower the computed RPS (shots go shorter).
     * 5) Re-test at a few distances to confirm the curve stays consistent.
     */
    public static final double VELOCITY_EFFICIENCY = 4.35;

    /** Fixed shooter launch angle in radians (used in the projectile motion calculation). */
    public static final double SHOOTER_ANGLE = Math.toRadians(68.0); // Convert 68 degrees to radians

    /** Height of the shooter exit point above the floor, in meters. */
    public static final double SHOOTER_HEIGHT = Units.inchesToMeters(28); // 27 inches to meters

    /** Height of the target center above the floor, in meters. */
    public static final double GOAL_HEIGHT = Units.inchesToMeters(72); // 72 inches to meters

    /** Vertical offset between the target and the shooter exit point (meters). */
    public static final double HEIGHT_DIFFERENCE = GOAL_HEIGHT - SHOOTER_HEIGHT;
  }

  /* ================= Intake ================= */

  public static final class IntakeConstants {
    private IntakeConstants() {} // Prevent instantiation

    /** CAN ID for the back intake motor. */
    public static final int INTAKE_MOTOR_ID = 29;
  }

  /* ================= Indexer ================= */

  public static final class IndexerConstants {
    private IndexerConstants() {} // Prevent instantiation

    /** CAN ID for the indexer motor. */
    public static final int INDEXER_MOTOR_ID = 28;
  }

  /* ================= Object Recognition ================= */

  public static final class ObjectRecognitionConstants {
    private ObjectRecognitionConstants() {} // Prevent instantiation

    /** The name of the limelight used for object detection. */
    public static final String LIMELIGHT_NAME = "limelight";

    /** Pipeline index for standard AprilTag processing. */
    public static final int APRIL_TAG_PIPELINE_INDEX = 0;

    /** Pipeline index for neural network object detection. */
    public static final int OBJECT_DETECTION_PIPELINE_INDEX = 1;

    /** The buffer for Limelight detection distance calculation, in seconds. */
    public static final double LIMELIGHT_TARGET_TIMEOUT = 0.2;
  }

  /* ================= Drivebase ================= */

  public static final class DrivebaseConstants {
    private DrivebaseConstants() {} // Prevent instantiation

    /** Time to hold wheel lock after disable in seconds. */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }

  /* ================= Operator Controls ================= */

  public static class OperatorConstants {
    private OperatorConstants() {} // Prevent instantiation

    /** Global joystick deadband to prevent stick drift. */
    public static final double DEADBAND = 0.1;

    /** Scalar applied to turning input for driver feel. */
    public static final double TURN_CONSTANT = 6.0;
  }
}
