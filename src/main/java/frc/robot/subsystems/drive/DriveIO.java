package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs {
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        public Rotation2d rotation2d = new Rotation2d();
    }


    public default void updateInputs(DriveIOInputs input)
}