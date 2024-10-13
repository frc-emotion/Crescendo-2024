// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Controls whether or not Debug Mode is activated. If it is, each Subsystem Shuffleboard tab
     * is supplied with most important subsystem info.
     */
    public static final boolean DEBUG_MODE_ACTIVE = true;

    /**
     * Contains controller configurations for the driver and operator
     */
    public static final class OIConstants {

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDeadband = 0.15;

        public static final double CLIMB_DEADZONE = 0.2;
        public static final double SHOOTER_DEADZONE = 0.2;
        public static final double PIVOT_DEADZONE = 0.2;
        public static final double INTAKE_DEADZONE = 0.2;
        public static final double kPersianSpeedMultiplier = 2.0;
        public static final double kSpeedDivideAdjustment = 2.0;
    }

    /**
     * Contains constants used during the Game
     */
    public static final class GameConstants {
        public static final double RED_NOTE_FEED_ANGLE = -32; // Used to shoot notes into the amp area during teleop
        public static final double BLUE_NOTE_FEED_ANGLE = 32; // Used to shoot notes into the amp area during teleop
    }

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(
                4);
        public static final double kDriveMotorGearRatio = 1.0 / 6.75;
        public static final double kTurningMotorGearRatio = 7.0 / 150.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.3;
        public static final double kDTurning = 0;

        public static final double drivekS = 0;
        public static final double drivekV = 0;
        public static final double drivekA = 0;
    }

    public static final class DriveConstants {
        public static enum DriveMode {
            NORMAL,
            TURBO,
            SLOW
        }

        public static DriveMode currentDriveMode = DriveMode.NORMAL;
        public static boolean isRobotCentric = false;

        public static final double kTrackWidth = Units.inchesToMeters(24.0);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.0);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        /** CAN IDs for all drive motors */
        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kBackLeftDriveMotorPort = 9;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 16;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 10;
        public static final int kBackRightTurningMotorPort = 5;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3; // 2143
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        /**
         * Calculate by Positioning wheels at zero manually and reading absolute encoder
         * values
         */

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = (19.42 - 0.1) / 360;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (-35.94 - 0.2 + 0.3) / 360;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (-89.03 - 1.5 + 0.2) / 360;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (64.16 - 0.8) / 360;
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(180 + 0.5);
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(180 + 39.8 - 3.1);
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(180 + -154.098 - 0.7);
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(157.7 - 4.7);
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(-154.098 - 0.7);
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(180 + 0.5);
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(157.7 - 4.7);
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
        // -Units.degreesToRadians(39.8 - 3.1);

        public static final double kPhysicalMaxSpeedMetersPerSecond = // 4.96 m/s
                6380.0 /
                        60.0 *
                        (14.0 / 50.0) *
                        (27.0 / 17.0) *
                        (15.0 / 45.0) *
                        0.10033 *
                        Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */

        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond /
                Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

        // No longer needed since calculations are done within code
        // public static final double kTeleDriveSuperSlowSpeedMetersPerSecond =
        // kPhysicalMaxSpeedMetersPerSecond / 6;
        // public static final double kTeleDriveSuperSlowlAngularSpeedRadiansPerSecond =
        // kPhysicalMaxAngularSpeedRadiansPerSecond / 6;

        // public static final double kTeleDriveSlowSpeedMetersPerSecond =
        // kPhysicalMaxSpeedMetersPerSecond / 4;
        // public static final double kTeleDriveSlowlAngularSpeedRadiansPerSecond =
        // kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        // public static final double kTeleDriveNormalSpeedMetersPerSecond =
        // kPhysicalMaxSpeedMetersPerSecond / 2;
        // public static final double kTeleDriveNormalAngularSpeedRadiansPerSecond =
        // kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        // Don't change these unless you want to increase max mps
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;

        // change acceleration based on drive testing
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kTeleDriveSlowAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveSlowAngularAccelerationUnitsPerSecond = 1.5;

        public static final double TARGET_ANGLE = Units.degreesToRadians(1);
        public static final double MAX_LEVEL_VELOCITY = 0.1; // FIX mps
        public static final double MAX_LEVEL_ACCELERATION = 0.3; // Fix with testing
        public static final double THRESHOLD = Units.degreesToRadians(3);

        public static final double kPThetaController = 0.2;
        public static final double kIThetaController = 0.00;
        public static final double kDThetaController = 0.00;
    }

    public static class AutoConstants {
        public static final boolean PATH_LOGGING = true;

        // Chane all values based on testing
        public static final double kMaxSpeedMetersPerSecond = 5.00;

        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Increase speed of rotation
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // Change based on testing/tune PID controllers
        public static final double kPXController = 3.56 / 2.0;
        public static final double kPYController = 0.75;

        public static final double kPThetaController = 0.52;
        public static final double kIThetaController = 0.045;
        public static final double kDThetaController = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints( //
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final double INTAKE_TIMEOUT = 0;

        public static final double SCORE_SPEAKER_TIMEOUT = 6.0;

        public static final Translation2d[] NOTE_POSITIONS = {
                new Translation2d(0, 0)
        };

        public static final Pose2d SCORE_SPEAKER_POSE = new Pose2d();

        public static final double SPEAKER_TAG_HEIGHT = 1.324;
        public static final double SPEAKER_MOUTH_HEIGHT = 2.083;
        public static final double PIVOT_HEIGHT = 0.371; // meters
        public static final double STAGE_TO_SPEAKER_THETA = 32.26; // degrees

        public static final boolean INITIAL_PLANNING_ENABLED = false;
        public static final boolean DYNAMIC_PLANNING_ENABLED = false;
        public static final double PLANNING_TOTAL_ERROR_THRESHOLD = 0;
        public static final double PLANNING_SPIKE_ERROR_THRESHOLD = 0;

    }

    public static final class ClimbConstants {

        public static final int CLIMB_PORT_L = 11;
        public static final int CLIMB_PORT_R = 18;

        // PID Constants (not used)
        public static final double EXTENSION_LIMIT = 140.0;
        public static final int PID_MIN_OUTPUT = 0;
        public static final int PID_MAX_OUTPUT = 0;
        public static final int SLOT_ID = 0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double MAX_ACCELERATION = 0.0;
        public static final double MAX_VELOCITY = 0.0;


        // Motor Constants
        public static final int CURRENT_LIMIT = 45;
        public static final int SMART_MAX_CURRENT = 40;

        // Input Deadzone
        public static final Double DEADZONE = 0.2;

        // Raw Output Speed
        public static final double kSpeed = 0.8;
    }

    public static class ShooterConstants {

        // CAN IDs and Ports
        public static final int feederPort = 22;
        public static final int shooterPort = 21;
        public static final int BREAK_SENSOR_PORT = 9;

        // Motor Constants
        public static final int CURRENT_LIMIT = 45; // amps
        public static final int CURRENT_LIMIT_SMART = 40; // amps
        public static final double GEAR_REDUCTION = 0.5;

        // PID Controller Constants
        public static final double kP = 0.0015;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFeedForward = 0.00025;
        public static final double kMaxOutput = 1; // raw motor output
        public static final double kMinOutput = -1; // raw motor output
        public static final double kMaxOutputError = 150; // rpm

        // Speeds
        public static final double kFeedSpeed = 0.35;
        public static final double SHOOTER_SPEED_RPM = 3500.0 + 500.0;
        public static final double[] PRESET_SPEEDS = { 3500 + 500}; // rpm
        public static final double IDLE_SPEED = 1000; // rpm
        public static final double AmpRPM = 1200;

        public static final double SHOOTER_FEEDER = 2500.0 + 500.0;

        // For SOURCE intake
        protected static final double SHOOTER_REVERSE_SPEED = -0.25; // raw motor output
        protected static final double FEEDER_REVERSE_SPEED = -0.175; // raw motor output
        public static final double MIN_SHOOT_SPEED = 2750;

        public static final double FLYWHEEL_MASS = 0.117; // kg
        public static final double FLYWHEEL_RADIUS = 0.051; // meters
        public static final double FLYWHEEL_MOI = 0.5 * FLYWHEEL_MASS * Math.pow(FLYWHEEL_RADIUS, 2);
        public static final double NOTE_MASS = 0.232; // kg
        
    }

    public static final class PivotConstants {

        // CAN IDs and Ports
        public static final int PIVOT_PORT = 20;
        public static final int absoluteEncoderID = 69;

        // PID Constants
        public static final double PIVOT_KP = 10.0;
        public static final double PIVOT_KI = 0;
        public static final double PIVOT_KD = 4;
        public static final double PIVOT_KF = 0;

        // Motor Constants
        public static final int MAX_CURRENT = 45;
        public static final int MAX_CURRENT_SMART = 40;
        public static final double MAX_ERROR = 1.5;
        public static final double GEAR_REDUCTION = 75.0;
        public static final double kConversionFactor = (-1.0 / PivotConstants.GEAR_REDUCTION) * 360;

        // Raw Output Speeds
        public static final double PIVOT_TELEOP_SPEED = 0.3;
        public static final double PIVOT_AUTO_SPEED = 0.3;
        public static final double PIVOT_ZERO_SPEED = 0.1;

        // Preset Positions
        public static final double PIVOT_MIN_REVOLUTION = 5;
        public static final double PIVOT_MAX_REVOLUTION = 83;
        public static final double PIVOT_THRESHOLD = 6;
        public static final double[] PIVOT_POSITIONS = { 65.0, 0 + 5};

        // Input Constants
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Extra Constants
        public static final double CURRENT_SPIKE_THRESHOLD = 0;

        // Handoff
        public static final double kHANDOFF_ANGLE = 60.0;
        public static final double kMAX_ANGLE_ERROR = 1.5;
    }

    public static final class IntakeConstants {

        // CAN IDs and Ports
        public static final int INTAKE_PIVOT_PORT_LEFT = 29;
        public static final int INTAKE_MOTOR_PORT = 19;
        public static final int INTAKE_PIVOT_PORT_RIGHT = 15;
        public static final int BEAM_BREAKER_PORT = 8;

        // Motor Constants
        public static final int MAX_CURRENT = 45;
        public static final int SMART_MAX_CURRENT = 40;
        public static final double CURRENT_SPIKE_THRESHOLD = 5.2;
        public static final double INTAKE_MOTOR_SPEED = 0.40;
        public static final double INTAKE_PIVOT_SPEED = 0.15;
        public static final double GEAR_REDUCTION = 43.5;

        /*
         * public static final double MAX_POSITION = 0;
         * public static final double MIN_POSITION = 0;
         * public static final double INTAKE_DOWN_POSITION = 0;
         * public static final double INTAKE_UP_POSITION = 0;
         */

        // Intake Pivot Constants
        // TODO: Retune PID Constants
        public static final double kP_PIVOT = 4.7;
        public static final double kI_PIVOT = 0;
        public static final double kD_PIVOT = 0;

        public static final double kPR_PIVOT = 4.7;
        public static final double kIR_PIVOT = 0;
        public static final double kDR_PIVOT = 0;

        // Input Constants (Unused)
        // private static final double kMaxVelocityDeg = 20;
        // private static final double kMaxAccelDeg = 5;
        // private static final double kMaxErrorDeg = 3;

        // private static final double DEPLOYED_POS_DEG = -13.5;
        // private static final double RETRACTED_POS_DEG = 0;

        // Do not edit these constants -------------

        // Actual Speed Constants
        public static final double kMaxVelocity = 2;
        public static final double kMaxAccel = 3;
        public static final double kMaxError = 1;

        public static final double DEPLOYED_POS = -0.3; // 0.3
        public static final double RETRACTED_POS = 0;

        public static final double SHOOTER_TRANSFER_SPEED = 0.35;
        public static final double AUTO_SHOOTER_TRANSFER_SPEED = 0.5;

        // -----------------------------------------

        /**
         * // TODO: FIX THIS METHOD!!!!
         *
         * @param degrees The target number of degrees
         * @return The equivalent measurement in meters
         */
        // private static double pivotDegreesToRevolutions(double degrees) {
        // return (degrees / 360.0) / GEAR_REDUCTION;
        // }
    }

    public static final class VisionConstants {
        public static final double FIELD_LENGTH = 16.541;
        public static boolean VISION_DEBUG_MODE = true;

        public static final Matrix<N3, N1> kTestStdDevs = VecBuilder.fill(0.2, 0.2, 1);
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final double TAG_DETECTION_THRESHOLD = Units.feetToMeters(15);

        // Thank you super nurds for constnats very nice (nice spelling)
        public static final Translation2d BLUE_SPEAKER_CENTER = new Translation2d(0.457 / 2, 5.557034);
        public static final Translation2d RED_SPEAKER_CENTER = new Translation2d(FIELD_LENGTH - (0.457 / 2), 5.557034);
    }

    public static final class LEDConstants {
        public static final Color INTAKE_COLOR = new Color(255, 50, 0); // orange
        public static final Color SHOOTER_COLOR = Color.kGreen;
        public static final Color RED_ALLIANCE_COLOR = Color.kRed;
        public static final Color BLUE_ALLIANCE_COLOR = Color.kBlue;
        public static final Color DISABLED_COLOR = Color.kYellow;

        public static final int FLASH_DELAY = 100;
    }

}
