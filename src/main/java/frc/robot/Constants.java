// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class OI {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEAD_BAND = 0.05;
  }

  public static final class CanbusId {
    // Drive motors
    public static final int FRONT_LEFT_DRIVE = 1, FRONT_LEFT_TURNING = 2;
    public static final int REAR_LEFT_DRIVE = 4, REAR_LEFT_TURNING = 5;
    public static final int FRONT_RIGHT_DRIVE = 7, FRONT_RIGHT_TURNING = 8;
    public static final int REAR_RIGHT_DRIVE = 10, REAR_RIGHT_TURNING = 11;
    // Intake motors
    public static final int INTAKE_LEFT = 13, INTAKE_RIGHT = 14, INTAKE_TOP = 15;
    // Pivot motors
    public static final int PIVOT_LEADER = 16, PIVOT_FOLLOWER = 17;
  }

  public static final class Pivot {
    public static final double P = 0.011, I = 0.0000000045, D = 0.0000000001, IZONE = 0.0, FF = 0.000156;
    public static final double MIN_OUTPUT = -0.2, MAX_OUTPUT = 0.15;
  }

  public static class Vision {
    public static final String CAMERA_NAME = "YOUR CAMERA NAME";

    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d());
  }

  public static final class Drive {
    // Meters per second
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;

    // Radians per second
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    // Radians per second
    public static final double DIRECTION_SLEW_RATE = 1.2;

    // Percent per second (1 = 100%)
    public static final double MAGNITUDE_SLEW_RATE = 1.8;

    // Percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 2.0;

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(17.5);

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(17.5);

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;
  }

  public static final class SwerveModule {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotor.kFreeSpeedRpm / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREEE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION; // Meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION) / 60.0; // Meters per second

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // Radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // Radians

    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREEE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double TURNING_P = 1;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    public static final IdleMode DRVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
  }

  public static final class Autonomous {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELARATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double PX_CONTROLLER = 1;
    public static final double PY_CONTROLLER = 1;
    public static final double THETA_CONTROLLER = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static final class NeoMotor {

    public static final double kFreeSpeedRpm = 5676;
  }
}
