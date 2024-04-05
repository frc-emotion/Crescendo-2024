// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.Auto.NamedCommands.CommandContainer;
import frc.robot.commands.Auto.NamedCommands.ShootSpeaker;
import frc.robot.commands.Auto.SubsystemCommands.AutoSpeakerTurret;
import frc.robot.commands.Auto.SubsystemCommands.HandoffAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.LEDAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.PivotAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.RevShooterAutoCommand;
import frc.robot.commands.Teleop.*;
import frc.robot.commands.Teleop.swerve.*;
import frc.robot.commands.debug.ResetGyroCommand;
import frc.robot.commands.vision.*;
import frc.robot.commands.vision.MonitorVision;
import frc.robot.commands.vision.SpeakerTurret;
import frc.robot.subsystems.*;
import frc.robot.util.AutoManager;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

import java.sql.Driver;
import java.util.Map;

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
    public static final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(
            m_SwerveSubsystem);
    public static final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

    private final AutoManager autoManager;

    // Command controllers used for Triggers
    public static final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    public static final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);

    // Initializes the controller HIDs, which are used for direct input, primarily
    // in the Default Commands.
    public static final XboxController driverController_HID = m_driverController.getHID();
    public static final XboxController operatorController_HID = m_operatorController.getHID();

    // The Auto SendableChooser
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
                        () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband));

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
                        () -> -operatorController_HID.getLeftY()
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

        // m_ledSubsystem.setDefaultCommand(
        // new LEDCommand(
        // m_ledSubsystem,
        // m_IntakeSubsystem::getBeamState,
        // m_IntakeSubsystem::isDown,
        // m_ShooterSubsystem::isAtTarget
        // )
        // );

        m_VisionSubsystem.setDefaultCommand(
                new MonitorVision(m_VisionSubsystem));

        registerNamedCommands();
        configureAutoChooser();

        // Configure the trigger bindings
        configureBindings();
        initializeAutoShuffleboard();
        initializeGameShuffleboard();
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Adds all the choices to the AutoChooser
     */
    private void configureAutoChooser() {
        // autoChooser.addOption("Simple Shoot", new ShootSpeaker(m_ShooterSubsystem));
        addOption("4 Note Auto");
        addOption("4 Note Angle Auto");
        addOption("4 Note Angle Auto No Turn");
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

        addOption("Forward Test Auto");
        addOption("Jank Test Auto");
        addOption("Turn Test Auto");
        addOption("Strafe Test Auto");
        // addOption("3 Note Top");

        // addOption("Note Push Top");
        // addOption("Note Push Bottom");
        addOption("1 Note Stationary");
        addOption("Note Push");
    }

    /**
     * Adds another option to the Auto Chooser using PathPlanner's
     * AutoBuilder to retrieve the Auto Command.
     * 
     * @param name The name of the PathPlanner Auto Command
     */
    private void addOption(String name) {
        autoChooser.addOption(name, AutoBuilder.buildAuto(name));
    }

    /**
     * Adds another option to the Auto Chooser.
     * 
     * @param name    The name to display in the Auto Chooser
     * @param command The Auto Command to run using this option
     */
    @SuppressWarnings("unused")
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
        // Swerve Slow Drive Mode
        m_driverController
                .leftBumper()
                .whileTrue(
                        new SlowModeSwerveCommand(
                                m_SwerveSubsystem,
                                () -> driverController_HID.getLeftY(),
                                () -> driverController_HID.getLeftX(),
                                () -> driverController_HID.getRightX(),
                                () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband));

        // Swerve Turbo Drive Mode
        m_driverController
                .rightBumper()
                .whileTrue(
                        new TurboModeSwerveCommand(
                                m_SwerveSubsystem,
                                () -> driverController_HID.getLeftY(),
                                () -> driverController_HID.getLeftX(),
                                () -> driverController_HID.getRightX(),
                                () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband));

        // Swerve Zero Heading
        m_driverController
                .b()
                .onTrue(
                        new InstantCommand() {
                            @Override
                            public void execute() {
                                m_SwerveSubsystem.zeroHeading();
                            }
                        });

        // Swerve Zero Heading
        m_driverController.b().onTrue(new InstantCommand() {
            @Override
            public void execute() {
                m_SwerveSubsystem.zeroHeading();
            }
        });

        m_driverController.leftTrigger(OIConstants.kDeadband).whileTrue(
                new ParallelCommandGroup(
                        new SnapSwerveCommand(
                                m_SwerveSubsystem,
                                () -> driverController_HID.getLeftY(),
                                () -> driverController_HID.getLeftX(),
                                () -> driverController_HID.getRightX(),
                                0
                        ),
                        new PivotAutoCommand(m_PivotSubsystem, 1)
                )
        );
  

        // Amp Shooting
        m_operatorController.rightBumper().whileTrue(
                new Command() {
                    public void execute() {
                        m_ShooterSubsystem.setShooterVelocity(
                                m_ShooterSubsystem.getAmpRPM());
                    }
                });

        // m_operatorController.povDown()
        // // .or(m_operatorController.povUp())
        // .onTrue(new InstantCommand() {
        // @Override
        // public void execute() {
        // int index = m_PivotSubsystem.getIndex();
        // m_ShooterSubsystem.setTargetRPM(ShooterConstants.PRESET_SPEEDS[index]);
        // }
        // });

        // m_operatorController.x().onTrue(
        // new IntakePivotCommand(m_IntakeSubsystem)
        // );

        // Intake Pivot Command
        m_operatorController
                .x()
                // .povUp()
                .whileTrue(
                        new SequentialCommandGroup(
                                new IntakePivotCommand(m_IntakeSubsystem)
                                        .onlyIf(() -> m_IntakeSubsystem.isUp()), // should
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
                .whileFalse(
                        // new SequentialCommandGroup(
                        new IntakePivotCommand(m_IntakeSubsystem)
                                .onlyIf(() -> !m_IntakeSubsystem.isUp())
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
                                        // new TeleopHandoffCommand(
                                        //         () -> operatorController_HID.getRightTriggerAxis() > OIConstants.kDeadband,
                                        //         m_ShooterSubsystem,
                                        //         m_IntakeSubsystem
                                                
                                        // )
                                        //         // .onlyIf(() -> m_PivotSubsystem.isHandoffOk())
                                        //         .withTimeout(2.0)
                                        new HandoffAutoCommand(m_IntakeSubsystem, m_ShooterSubsystem, false).withTimeout(2.0)
                                )
                // )
                );

        // Resets the Pivot to default position
        m_operatorController
                .povDown()
                .onTrue(new PivotAutoCommand(m_PivotSubsystem, 1));

        // Direct Source Intake Mode
        m_operatorController
                .b()
                .whileTrue(
                        new Command() {
                            @Override
                            public void execute() {
                                m_ShooterSubsystem.setShooterRaw(
                                        ShooterConstants.SHOOTER_REVERSE_SPEED);
                                m_ShooterSubsystem.setFeederSpeed(
                                        ShooterConstants.FEEDER_REVERSE_SPEED);
                            }

                            @Override
                            public void end(boolean interrupted) {
                                m_ShooterSubsystem.stopShooter();
                                m_ShooterSubsystem.stopFeeder();
                            }
                        });

        // Handoff Manual Mode
        m_operatorController.a().whileTrue(
                new TeleopHandoffCommand(
                        () -> operatorController_HID.getRightTriggerAxis() > OIConstants.kDeadband,
                        m_ShooterSubsystem, m_IntakeSubsystem
                 )
        );

        // Climb Encoder Reset Command
        m_operatorController.start().onTrue(new InstantCommand() {
            @Override
            public void execute() {
                m_PivotSubsystem.resetPosition(PivotConstants.PIVOT_POSITIONS[0]);
            }
        });

        m_operatorController.y().whileTrue(
                // new SpeakerTurret(m_VisionSubsystem, m_PivotSubsystem)
                // new ParallelCommandGroup(
                //         new SnapCommand(m_SwerveSubsystem, m_VisionSubsystem), 
                //         new SpeakerTurret(m_VisionSubsystem, m_PivotSubsystem)
                //         )
                        new PivotAutoCommand(m_PivotSubsystem, 3)
                );

    }

    /**
     * Registers all the NamedCommands for use with PathPlanner.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand(
                "ScoreSpeaker",
                new ShootSpeaker(m_ShooterSubsystem));
        NamedCommands.registerCommand(
                "IntakeNote",
                new IntakeDriveAutoCommand(m_IntakeSubsystem));
        NamedCommands.registerCommand(
                "ResetPivot",
                CommandContainer.resetPivot(m_PivotSubsystem));
        NamedCommands.registerCommand(
                "ToggleIntake",
                new IntakePivotCommand(m_IntakeSubsystem));
        NamedCommands.registerCommand(
                "RevShooter",
                new RevShooterAutoCommand(m_ShooterSubsystem));
        NamedCommands.registerCommand(
                "AutoHandoff",
                // CommandContainer.getAutoHandoffCommandGroup(
                //         m_IntakeSubsystem,
                //         m_ShooterSubsystem,
                //         m_PivotSubsystem));
                new HandoffAutoCommand(m_IntakeSubsystem, m_ShooterSubsystem, true).withTimeout(3.0));
        NamedCommands.registerCommand(
                "AutoHandoffNoTimeout",
                new HandoffAutoCommand(m_IntakeSubsystem, m_ShooterSubsystem, true).onlyIf(() -> !m_IntakeSubsystem.getBeamState())
        );
        NamedCommands.registerCommand(
                "PrepPivot",
                new AutoSpeakerTurret(m_VisionSubsystem, m_PivotSubsystem));
    }

    /**
     * Sends all necessary data to the GAME tab in Shuffleboard and adds all
     * widgets.
     */
    private void initializeGameShuffleboard() {
        ShuffleboardTab gameShuffleboardTab = TabManager
                .getInstance()
                .accessTab(SubsystemTab.GAME);

        // Generic Data - The heading, whether or not debug mode is active, and the auto
        // chooser.

        gameShuffleboardTab
                .addNumber("Robot Heading", () -> m_SwerveSubsystem.getHeading())
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(10, 0)
                .withSize(3, 4);
        gameShuffleboardTab
                .addBoolean("Debug Mode", () -> Constants.DEBUG_MODE_ACTIVE)
                .withPosition(10, 4)
                .withSize(3, 1);
        // gameShuffleboardTab.add("Auto Chooser", autoChooser).withPosition(8,
        // 2).withSize(2, 1);

        // Drive Layout - Shows which drive mode is active (Slow, Normal, Turbo)
        ShuffleboardLayout generalLayout = gameShuffleboardTab
                .getLayout("Drive Mode", BuiltInLayouts.kGrid)
                .withPosition(5, 0)
                .withProperties(Map.of("Number of columns", 4, "Number of Rows", 1))
                .withSize(5, 2);
        generalLayout
                .addBoolean(
                        "Slow Mode Active",
                        () -> DriveConstants.currentDriveMode == DriveMode.SLOW)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(
                        Map.of(
                                "Color when false",
                                "#808080",
                                "Color when true",
                                "#FFFF4"));
        generalLayout
                .addBoolean(
                        "Normal Mode Active",
                        () -> DriveConstants.currentDriveMode == DriveMode.NORMAL)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when false", "808080"));
        generalLayout
                .addBoolean(
                        "Turbo Mode Active",
                        () -> DriveConstants.currentDriveMode == DriveMode.TURBO)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(
                        Map.of(
                                "Color when false",
                                "#808080",
                                "Color when true",
                                "#0000FF"));
        generalLayout
                .addBoolean(
                        "Robot Centric Mode Active",
                        () -> DriveConstants.isRobotCentric)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(
                        Map.of(
                                "Color when false",
                                "#808080",
                                "Color when true",
                                "#FF00FF"));

        // Intake Layout - Shows the deployed state of the intake and if the beam sensor
        // is detecting something
        ShuffleboardLayout intakeLayout = gameShuffleboardTab
                .getLayout("Intake Data", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of Rows", 1))
                .withPosition(0, 0)
                .withSize(5, 2);
        intakeLayout
                .addBoolean("Intake Down", () -> !m_IntakeSubsystem.isUp())
                .withWidget(BuiltInWidgets.kBooleanBox);
        intakeLayout
                .addBoolean("Beam Broken", () -> m_IntakeSubsystem.getBeamState())
                .withWidget(BuiltInWidgets.kBooleanBox);
        gameShuffleboardTab
                .addNumber(
                        "Shooter RPM",
                        () -> m_ShooterSubsystem.getShooterVelocity())
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(0, 3)
                .withSize(3, 3);

        // gameShuffleboardTab.addBoolean("Shooter At Speed", () ->
        // m_ShooterSubsystem.getShooterVelocity() > 3900)
        // .withPosition(3, 3).withSize(2, 2);
        // gameShuffleboardTab.addBoolean("Shooter Beam State", () ->
        // m_ShooterSubsystem.isProjectileFed())
        // .withPosition(3, 5).withSize(2, 1);

        // gameShuffleboardTab.add("Robot Pose", () ->
        // m_SwerveSubsystem.getCurrentPose()).withWidget(BuiltInWidgets.kField).withPosition(4,
        // 6);

        ShuffleboardLayout motorLayout = gameShuffleboardTab
                .getLayout("Motor Data", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1))
                .withPosition(8, 3)
                .withSize(2, 1);
        motorLayout
                .addNumber(
                        "Shooter Temp",
                        () -> m_ShooterSubsystem.getShooterTemp())
                .withWidget(BuiltInWidgets.kTextView);
        motorLayout
                .addNumber(
                        "Intake Drive Temp",
                        () -> m_IntakeSubsystem.getIntakeDriveTemp())
                .withWidget(BuiltInWidgets.kTextView);
        // Sets GAME to active tab
        // Shuffleboard.selectTab("GAME");

        gameShuffleboardTab.add(
                "Pivot Angle",
                new SpeakerTurret(m_VisionSubsystem, m_PivotSubsystem));
    }

    /**
     * Sends the data to the AUTO tab in Shuffleboard
     */
    private void initializeAutoShuffleboard() {
        ShuffleboardTab autoTab = TabManager
                .getInstance()
                .accessTab(SubsystemTab.AUTO);

        //autoTab.add("Angle to Speaker", new Rotate(m_SwerveSubsystem, m_VisionSubsystem, autoManager));

        autoTab.add(autoChooser).withSize(3, 1);
        autoTab
                .add("Auto Visualizer", autoManager.getAutoField2d())
                .withWidget(BuiltInWidgets.kField)
                .withSize(4, 3);

        ShuffleboardLayout matchLayout = autoTab
                .getLayout("Match Data", BuiltInLayouts.kGrid)
                .withProperties(
                        Map.of(
                                "Number of columns",
                                3,
                                "Number of rows",
                                1,
                                "Label Position",
                                "TOP"))
                .withSize(4, 1);
        matchLayout.addString("Event Name", () -> DriverStation.getEventName());
        matchLayout.addString(
                "Match Type",
                () -> DriverStation.getMatchType().name());
        matchLayout.addNumber(
                "Match Number",
                () -> DriverStation.getMatchNumber());
        autoTab
                .add("Reset Heading", ResetGyroCommand.getCommand())
                .withWidget(BuiltInWidgets.kCommand)
                .withSize(2, 2);
    }

    /**
     * Retrieves the selected Auto from the Auto Chooser.
     * 
     * @return The currently selected Auto Command
     */
    public Command getAutonomousCommand() {
        try {
                return new ParallelCommandGroup(
                        autoChooser.getSelected(),
                        new LEDAutoCommand(m_ledSubsystem),
                        new MonitorVision(m_VisionSubsystem)
                );
        } catch(Exception ex) {
                DriverStation.reportError("Autonomous Command Scheduling Error", true);
                return new ShootSpeaker(m_ShooterSubsystem);
        }
        
    }

    public Command getLEDCommand() {
        return new LEDCommand(
                m_ledSubsystem,
                m_IntakeSubsystem::getBeamState,
                m_IntakeSubsystem::isDown,
                m_ShooterSubsystem::isProjectileFed,
                () -> (( m_ShooterSubsystem.getShooterVelocity() > ShooterConstants.MIN_SHOOT_SPEED))
                        // || (driverController_HID.getRightBumper() > OIConstants.kDeadband && m_ShooterSubsystem.getShooterVelocity() > ShooterConstants.AmpRPM - 250))
                );
    }

    public Command getControllerRumbleCommand() {
        return new ControllerRumbleCommand(
                driverController_HID,
                operatorController_HID,
                () -> m_IntakeSubsystem.getBeamState(),
                () -> m_IntakeSubsystem.isDown(),
                () -> m_ShooterSubsystem.getShooterVelocity() > m_ShooterSubsystem.getAmpRPM() -
                        200 &&
                        operatorController_HID.getRightBumper(),
                () -> m_ShooterSubsystem.getShooterVelocity() > ShooterConstants.MIN_SHOOT_SPEED &&
                        operatorController_HID.getRightTriggerAxis() > OIConstants.kDeadband);
    }
}
