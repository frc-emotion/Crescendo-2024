// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.subsystems.*;

import frc.robot.commands.Auto.NamedCommands.*;
import frc.robot.commands.Auto.SubsystemCommands.*;
import frc.robot.commands.Teleop.*;
import frc.robot.commands.Teleop.swerve.*;
import frc.robot.commands.vision.*;

import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.DriveMode;

import frc.robot.util.*;
import frc.robot.util.TabManager.SubsystemTab;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    public static final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
    public static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public static final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    public static final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
    public static final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_SwerveSubsystem);

    private final AutoManager autoManager;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    public static final XboxController driverController_HID = m_driverController.getHID();

    public static final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);

    public static final XboxController operatorController_HID = m_operatorController.getHID();

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autoManager = AutoManager.getInstance();

        m_SwerveSubsystem.setDefaultCommand(
                new DefaultSwerveXboxCommand(
                        m_SwerveSubsystem,
                        () -> driverController_HID.getLeftY(),
                        () -> driverController_HID.getLeftX(),
                        () -> driverController_HID.getRightX(),
                        () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband

                ));

        m_ShooterSubsystem.setDefaultCommand(
                new ShooterManualCommand(
                        () -> operatorController_HID.getLeftTriggerAxis() > OIConstants.kDeadband,
                        () -> operatorController_HID.getRightTriggerAxis() > OIConstants.kDeadband,
                        m_ShooterSubsystem));

        m_ClimbSubsystem.setDefaultCommand(
                new ClimbManualCommand(
                        m_ClimbSubsystem,
                        () -> -operatorController_HID.getRightY()));

        m_PivotSubsystem.setDefaultCommand(
                new PivotManualCommand(
                        m_PivotSubsystem,
                        () -> operatorController_HID.getLeftY()
                // () -> operatorController_HID.getLeftStickButton()
                // () -> operatorController_HID.getPOV() == 0,
                // () -> operatorController_HID.getPOV() == 180,
                // () -> operatorController_HID.getPOV() == 90
                ));

        // m_IntakeSubsystem.setDefaultCommand(
        // new IntakeDriveCommand(
        // m_IntakeSubsystem,
        // () -> operatorController_HID.getLeftBumper(),
        // () -> operatorController_HID.getRightBumper()
        // )
        // );

        m_IntakeSubsystem.setDefaultCommand(
                new IntakeDriveCommand(
                        m_IntakeSubsystem,
                        () -> false, // previous r bumper
                        () -> operatorController_HID.getLeftBumper()));

        m_VisionSubsystem.setDefaultCommand(new MonitorVision(m_VisionSubsystem));
        registerNamedCommands();
        configureAutoChooser();

        // Configure the trigger bindings
        configureBindings();
        initializeAutoShuffleboard();
        initializeGameShuffleboard();

        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Auto Chooser Methods

    private void configureAutoChooser() {
        // autoChooser.addOption("Simple Shoot", new ShootSpeaker(m_ShooterSubsystem));
        addOption("4 Note Auto");
        addOption("3 Note Top Travel");
        addOption("3 Note Mid Top");
        addOption("3 Note Mid Bottom");
        addOption("3 Note Bottom Travel");
        // addOption("1 Note Stationary");  
        // addOption("1 Note Top");
        // addOption("1 Note Mid");
        // addOption("1 Note Bottom");
        addOption("2 Note Top");
        addOption("2 Note Mid");
        addOption("2 Note Bottom");
        addOption("2 Note Bottom Far");
        
        // addOption("Forward Test Auto");
        // addOption("Jank Test Auto");
        // addOption("Turn Test Auto");
        // addOption("Strafe Test Auto");
        // addOption("3 Note Top");
        
        
        // addOption("Note Push Top");
        // addOption("Note Push Bottom");
        addOption("1 Note Stationary");
        addOption("Note Push"); 
        

    }

    private void addOption(String name) {
        autoChooser.addOption(name, AutoBuilder.buildAuto(name));
    }

    private void addOption(String name, Command command) {
        autoChooser.addOption(name, command);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

        // m_operatorController.a().onTrue(new InstantCommand() {
        // @Override
        // public void execute() {
        // m_PivotSubsystem.calibrate();
        // }
        // }
        // );

        // m_driverController.y().toggleOnTrue(
        // new SpeakerTurret(m_VisionSubsystem, m_PivotSubsystem)
        // );

        m_driverController.leftBumper().whileTrue(
                new SlowModeSwerveCommand(
                        m_SwerveSubsystem,
                        () -> driverController_HID.getLeftY(),
                        () -> driverController_HID.getLeftX(),
                        () -> driverController_HID.getRightX(),
                        () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband));

        m_driverController.rightBumper().whileTrue(
                new TurboModeSwerveCommand(
                        m_SwerveSubsystem,
                        () -> driverController_HID.getLeftY(),
                        () -> driverController_HID.getLeftX(),
                        () -> driverController_HID.getRightX(),
                        () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband));

        m_driverController.b().onTrue(new InstantCommand() {
            @Override
            public void execute() {
                m_SwerveSubsystem.zeroHeading();
            }
        });

        // Drive Snapping Setup
        for (int angle = 0; angle < 360; angle += 45) {
            m_driverController.pov(angle).onTrue(
                    new SnapSwerveCommand(
                            m_SwerveSubsystem,
                            () -> driverController_HID.getLeftY(),
                            () -> driverController_HID.getLeftX(),
                            () -> driverController_HID.getRightX(),
                            angle));
        }

        m_operatorController.rightBumper().whileTrue(
                new Command() {
                        public void execute() {
                                m_ShooterSubsystem.setShooterVelocity(m_ShooterSubsystem.getAmpRPM());
                                if(m_ShooterSubsystem.getShooterVelocity() > ShooterConstants.AmpRPM - 250) {
                                        operatorController_HID.setRumble(RumbleType.kBothRumble, 0.25);
                                } else {
                                        operatorController_HID.setRumble(RumbleType.kBothRumble, 0);
                                }
                        }
                });

        m_operatorController.povDown()
                // .or(m_operatorController.povUp())
                .onTrue(new InstantCommand() {
                    @Override
                    public void execute() {
                        int index = m_PivotSubsystem.getIndex();
                        m_ShooterSubsystem.setTargetRPM(ShooterConstants.PRESET_SPEEDS[index]);
                    }
                });

        // m_operatorController.x().onTrue(
        // new IntakePivotCommand(m_IntakeSubsystem)
        // );

        m_operatorController
                .x()
                // .povUp()
                .whileTrue(
                        new SequentialCommandGroup(
                                new IntakePivotCommand(m_IntakeSubsystem).onlyIf(() -> m_IntakeSubsystem.isUp()), // should
                                                                                                                  // we
                                                                                                                  // change
                                                                                                                  // this
                                                                                                                  // to
                                                                                                                  // .andThen()
                                                                                                                  // instead
                                                                                                                  // of
                                                                                                                  // sequential
                                                                                                                  // command
                                                                                                                  // group?
                                new IntakeDriveAutoCommand(m_IntakeSubsystem)))
                // .onFalse(
                // new IntakePivotCommand(m_IntakeSubsystem).onlyIf(()->
                // !m_IntakeSubsystem.isUp())
                // )
                .whileFalse(
                        // new SequentialCommandGroup(
                        new IntakePivotCommand(m_IntakeSubsystem).onlyIf(() -> !m_IntakeSubsystem.isUp())
                                .alongWith(
                                        CommandContainer.resetPivot(m_PivotSubsystem)
                                // new Command() {
                                // @Override
                                // public void execute() {
                                // m_PivotSubsystem.goToHandoff();
                                // }

                                // @Override
                                // public boolean isFinished() {
                                // return m_PivotSubsystem.isHandoffOk();
                                // }

                                // @Override
                                // public void end(boolean interrupted) {
                                // m_PivotSubsystem.stop();
                                // }
                                // }
                                // .onlyIf(()->!m_PivotSubsystem.isHandoffOk())
                                // .withTimeout(3.5)
                                )
                                .andThen(
                                        new HandoffAutoCommand(m_IntakeSubsystem, m_ShooterSubsystem, false)
                                                .onlyIf(() -> m_PivotSubsystem.isHandoffOk())
                                                .withTimeout(2.0))
                // )
                );

        m_operatorController.povDown().onTrue(
                new PivotAutoCommand(m_PivotSubsystem, 1));

        m_operatorController.b().whileTrue(
                new Command() {
                    @Override
                    public void execute() {
                        m_ShooterSubsystem.setShooterRaw(ShooterConstants.SHOOTER_REVERSE_SPEED);
                        m_ShooterSubsystem.setFeederSpeed(ShooterConstants.FEEDER_REVERSE_SPEED);
                    }

                    @Override
                    public void end(boolean interrupted) {
                        m_ShooterSubsystem.stopShooter();
                        m_ShooterSubsystem.stopFeeder();
                    }
                });

        m_operatorController.a().whileTrue(
                new HandoffAutoCommand(m_IntakeSubsystem, m_ShooterSubsystem, false));

        m_operatorController.start().onTrue(new InstantCommand() {
            @Override
            public void execute() {
                m_ClimbSubsystem.reset();
            }
        });
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ScoreSpeaker",
                new ParallelRaceGroup(new ShootSpeaker(m_ShooterSubsystem), new WaitCommand(3.5))); // .withTimeout(AutoConstants.SCORE_SPEAKER_TIMEOUT));
        NamedCommands.registerCommand("IntakeNote", CommandContainer.intakeNote(m_IntakeSubsystem));
        NamedCommands.registerCommand("ResetPivot", CommandContainer.resetPivot(m_PivotSubsystem));
        NamedCommands.registerCommand("ToggleIntake", new IntakePivotCommand(m_IntakeSubsystem));
        NamedCommands.registerCommand("RevShooter", new RevShooterAutoCommand(m_ShooterSubsystem));
        NamedCommands.registerCommand("AutoHandoff",
                CommandContainer.getHandoffCommand(m_IntakeSubsystem, m_ShooterSubsystem));
        NamedCommands.registerCommand("PrepPivot", new SpeakerTurret(m_VisionSubsystem, m_PivotSubsystem));
    }

    private void initializeGameShuffleboard() {
        ShuffleboardTab gameShuffleboardTab = TabManager
                .getInstance()
                .accessTab(SubsystemTab.GAME);

        // Generic Data - The heading, whether or not debug mode is active, and the auto
        // chooser.
        gameShuffleboardTab.addNumber("Robot Heading", () -> m_SwerveSubsystem.getHeading())
                .withWidget(BuiltInWidgets.kGyro).withPosition(10, 0).withSize(3, 4);
        gameShuffleboardTab.addBoolean("Debug Mode", () -> Constants.DEBUG_MODE_ACTIVE).withPosition(10, 4).withSize(3, 1);
        // gameShuffleboardTab.add("Auto Chooser", autoChooser).withPosition(8, 2).withSize(2, 1);

        // Drive Layout - Shows which drive mode is active (Slow, Normal, Turbo)
        ShuffleboardLayout generalLayout = gameShuffleboardTab.getLayout("Drive Mode", BuiltInLayouts.kGrid)
                .withPosition(5, 0).withProperties(Map.of("Number of columns", 4, "Number of Rows", 1)).withSize(5, 2);
        generalLayout.addBoolean("Slow Mode Active", () -> DriveConstants.currentDriveMode == DriveMode.SLOW)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when false", "#808080", "Color when true", "#FFFF4"));
        generalLayout.addBoolean("Normal Mode Active", () -> DriveConstants.currentDriveMode == DriveMode.NORMAL)
                .withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when false", "808080"));
        generalLayout.addBoolean("Turbo Mode Active", () -> DriveConstants.currentDriveMode == DriveMode.TURBO)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when false", "#808080", "Color when true", "#0000FF"));
        generalLayout.addBoolean("Robot Centric Mode Active", () -> DriveConstants.isRobotCentric)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when false", "#808080", "Color when true", "#FF00FF"));

        // Intake Layout - Shows the deployed state of the intake and if the beam sensor
        // is detecting something
        ShuffleboardLayout intakeLayout = gameShuffleboardTab.getLayout("Intake Data", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of Rows", 1)).withPosition(0, 0).withSize(5, 2);
        intakeLayout.addBoolean("Intake Down", () -> !m_IntakeSubsystem.isUp()).withWidget(BuiltInWidgets.kBooleanBox);
        intakeLayout.addBoolean("Beam Broken", () -> m_IntakeSubsystem.getBeamState())
                .withWidget(BuiltInWidgets.kBooleanBox);

        gameShuffleboardTab.addNumber("Shooter RPM", () -> m_ShooterSubsystem.getShooterVelocity())
                .withWidget(BuiltInWidgets.kGraph).withPosition(0, 3).withSize(3, 3);
        // gameShuffleboardTab.addBoolean("Shooter At Speed", () -> m_ShooterSubsystem.getShooterVelocity() > 3900)
        //         .withPosition(3, 3).withSize(2, 2);
        // gameShuffleboardTab.addBoolean("Shooter Beam State", () -> m_ShooterSubsystem.isProjectileFed())
        //         .withPosition(3, 5).withSize(2, 1);

        // gameShuffleboardTab.add("Robot Pose", () ->
        // m_SwerveSubsystem.getCurrentPose()).withWidget(BuiltInWidgets.kField).withPosition(4,
        // 6);

        ShuffleboardLayout motorLayout = gameShuffleboardTab.getLayout("Motor Data", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1)).withPosition(8, 3).withSize(2, 1);
        motorLayout.addNumber("Shooter Temp", () -> m_ShooterSubsystem.getShooterTemp())
                .withWidget(BuiltInWidgets.kTextView);
        motorLayout.addNumber("Intake Drive Temp", () -> m_IntakeSubsystem.getIntakeDriveTemp())
                .withWidget(BuiltInWidgets.kTextView);

        // Sets GAME to active tab
        // Shuffleboard.selectTab("GAME");
    }

    private void initializeAutoShuffleboard() {
        ShuffleboardTab autoTab = TabManager
                .getInstance()
                .accessTab(SubsystemTab.AUTO);

        autoTab.add(autoChooser).withSize(3, 1);
        autoTab.add("Auto Visualizer", autoManager.getAutoField2d()).withWidget(BuiltInWidgets.kField).withSize(4, 3);
        
        ShuffleboardLayout matchLayout = autoTab.getLayout("Match Data", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 3, "Number of rows", 1, "Label Position", "TOP")).withSize(4, 1);
        matchLayout.addString("Event Name", () -> DriverStation.getEventName());
        matchLayout.addString("Match Type", () -> DriverStation.getMatchType().name());
        matchLayout.addNumber("Match Number", () -> DriverStation.getMatchNumber());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}