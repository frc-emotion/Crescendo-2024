// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Add all Driver/Teleop Controller here
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        //Change to match driver button map 
        public static final int kDriverYAxis = 1;
        
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = XboxController.Button.kA.value;
        public static final int kDriverZeroHeadingButtonIdx = XboxController.Button.kB.value;
        public static final int kDriverSlowButtonIdx = XboxController.Button.kLeftBumper.value;
        public static final int kDriverTurboButtonIdx = XboxController.Button.kRightBumper.value;

        public static final double kDeadband = 0.08;
    }

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = (double) 7 / 150 ;
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

    /*public static final class CameraConstants {
        
        // currently Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        
        public static final double CAMERA_ROLL = 0;
        public static final double CAMERA_PITCH = 0;
        public static final double CAMERA_YAW = 0;

        public static final double CAMERA_XAXIS = 0.5;
        public static final double CAMERA_YAXIS = 0;
        public static final double CAMERA_ZAXIS = 0.5;
    } */


    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21.0);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.0);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                // new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                // new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                // new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2));


        /**CAN IDs for all drive motors */
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort =9;
        public static final int kBackLeftTurningMotorPort = 11;
        public static final int kFrontRightTurningMotorPort = 12;
        public static final int kBackRightTurningMotorPort = 10;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;  //2143
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;


        /**
         * Calculate by Positioning wheels at zero manually and reading absolute encoder values
         */
        
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + 0.5); 
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + 39.8 - 3.1);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + -154.098 - 0.7); 
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  -Units.degreesToRadians(157.7 - 4.7 + 12.351);
        //  public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + 0.5); 
        //  public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + 39.8 - 3.1);
        //  public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + -154.098 - 0.7); 
        //  public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  -Units.degreesToRadians(157.7 - 4.7);
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(-154.098 - 0.7);
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(180 + 0.5);
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(157.7 - 4.7);
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -Units.degreesToRadians(39.8 - 3.1);



        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * Math.PI;
      
        /**
        * The maximum angular velocity of the robot in radians per second.
        * <p>
        * This is a measure of how fast the robot can rotate in place.
        */


       // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
       public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond /
               Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);


        // No longer needed since calculations are done within code
        // public static final double kTeleDriveSuperSlowSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 6;
        // public static final double kTeleDriveSuperSlowlAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 6;

        // public static final double kTeleDriveSlowSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        // public static final double kTeleDriveSlowlAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        // public static final double kTeleDriveNormalSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        // public static final double kTeleDriveNormalAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        //Don't change these unless you want to increase max mps
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;

        //change acceleration based on drive testing
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kTeleDriveSlowAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveSlowAngularAccelerationUnitsPerSecond = 1.5;

        public static final double TARGET_ANGLE = Units.degreesToRadians(1);
        public static final double MAX_LEVEL_VELOCITY = 0.1; //FIX mps 
        public static final double MAX_LEVEL_ACCELERATION = 0.3; //Fix with testing
        public static final double THRESHOLD = Units.degreesToRadians(3);
        public static final double KPLevel = 5;
        public static final double KDLevel = 0.3;
        public static final double KILevel = 0;

    }

    public static class AutoConstants {

        //Chane all values based on testing
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        //Increase speed of rotation
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    
        //Change based on testing/tune PID controllers
        public static final double kPXController = 3.1;
        public static final double kPYController = 3.1;
        public static final double kPThetaController = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }


    public static final class CameraConstants {
       
        // currently Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final double CAMERA_ROLL = 0;
        public static final double CAMERA_PITCH = 0;
        public static final double CAMERA_YAW = 0;


        public static final double CAMERA_XAXIS = 0.24;
        public static final double CAMERA_YAXIS = 0;
        public static final double CAMERA_ZAXIS = 0;


        public static final double TARGET_RANGE = 1.0; //how far in front of the target we want to align to
       
        public static final double MAX_ALIGN_VELOCITY = 1; //FIX mps
        public static final double MAX_ALIGN_ACCELERATION = 3; //FIX ?
        public static final double MAX_ROTATE_VELOCITY = 1; //FIX  
        public static final double MAX_ROTATE_ACCELERATION = 3; //FIX
    }

    public static final class ArmConstants {
       
        public static final int armMotorPort = 22;
        public static final int absoluteEncoderPort = 9;


        public static final double ARM_SPEED = -0.3;
        public static final double DOWN_ARM_SPEED = -0.1;
        public static final double TOP_HEIGHT = Math.PI / 2;
        public static final double MIDDLE_HEIGHT = Math.PI / 4;
        public static final double LOW_HEIGHT = Math.PI / 8;


        // For Feedfoward
        public static final double armKS = 0.09;
        public static final double armKG = 0.27;
        public static final double armKV = 1.18;
        public static final double armKA = 0.51;


        // For PID controller
        public static final double armKP = 1.23;
        public static final double armKD = 0;
        public static final double armKI = 0;
        public static final double COUNTS_PER_REVOLUTION = 4096;
        public static final double MAX_ARM_VELOCITY = 0.1;
        public static final double MAX_ARM_ACCELERATION = 0.05;
        public static final double ARM_GEAR_RATIO = 36.56;
        public static final double ARM_SPROCKET_RADIUS = 0.01;


       
    }


    public static class ClawConstants {


    }

    public static final class ElevatorConstants{
       
        
    }

    public static class ShooterConstants {
            // CAN IDs
        public static final int feederPort = 0;
        public static final int shooterPort = 1;

            // Motor Constants
        public static final int CURRENT_LIMIT = 45;

            // Speeds
        public static final double FEED_SPEED = 0.25;
        public static final double SHOOT_SPEED = 0.75;

            // FeedForward Constants
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

            // Prevent FeedForward overspeeding
        public static final double kFeedForwardAdjustment = 0.9;

            // Tolerance for atSetpoint() is at target speed
        public static final double kControllerTolerance = 0;
    }

}