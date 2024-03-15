// package frc.robot.commands.Teleop.swerve;

// import java.util.function.Supplier;

// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class VisionSpeakerSwerveCommand extends SnapSwerveCommand {
//     private final VisionSubsystem visionSubsystem;

//     public VisionSpeakerSwerveCommand(
//             SwerveSubsystem swerveSubsystem,
//             Supplier<Double> xSpdFunc,
//             Supplier<Double> ySpdFunc,
//             Supplier<Double> turningSpdFunc,
//             VisionSubsystem visionSubsystem
//     ) {
//         super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc, 0);
//         this.visionSubsystem = visionSubsystem;
//     }

//     @Override
//     public void execute() {
//         super.execute();
//     }
// }