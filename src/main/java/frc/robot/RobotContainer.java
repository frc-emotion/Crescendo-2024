// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.*;
import frc.robot.util.AutoManager;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.Auto.NamedCommands.CommandContainer;
import frc.robot.commands.Auto.NamedCommands.ShootSpeaker;
import frc.robot.commands.Auto.SubsystemCommands.RevShooterAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.TimeDriveAuto;
import frc.robot.commands.Auto.SubsystemCommands.HandoffAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.ResetPivotAutoCommand;
import frc.robot.commands.Teleop.*;
import frc.robot.commands.Teleop.swerve.*;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

    public static final AutoManager autoManager = new AutoManager(m_VisionSubsystem, m_SwerveSubsystem);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
        OIConstants.kDriverControllerPort
    );

    private final XboxController driverController_HID = m_driverController.getHID();

    private final CommandXboxController m_operatorController = new CommandXboxController(
        OIConstants.kOperatorControllerPort
    );

    private final XboxController operatorController_HID = m_operatorController.getHID();

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        

        m_SwerveSubsystem.setDefaultCommand(
            new DefaultSwerveXboxCommand(
                m_SwerveSubsystem,
                () -> driverController_HID.getLeftY(),
                () -> driverController_HID.getLeftX(),
                () -> driverController_HID.getRightX(),
                () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband
                
            )
        );

         m_ShooterSubsystem.setDefaultCommand(
             new ShooterManualCommand(
                 () -> operatorController_HID.getAButton(),
                 () -> operatorController_HID.getRightTriggerAxis() > OIConstants.kDeadband,
                 m_ShooterSubsystem
             )
        );

        m_ClimbSubsystem.setDefaultCommand(
            new ClimbManualCommand(
                m_ClimbSubsystem, 
                () -> -operatorController_HID.getRightY()
                )
            );

         m_PivotSubsystem.setDefaultCommand(
             new PivotManualCommand(
                 m_PivotSubsystem,
                 () -> operatorController_HID.getLeftY()
                // () -> operatorController_HID.getLeftStickButton()
                //  () -> operatorController_HID.getPOV() == 0,
                //  () -> operatorController_HID.getPOV() == 180,
                //  () -> operatorController_HID.getPOV() == 90
                 )
         );

        // m_IntakeSubsystem.setDefaultCommand(
        //     new IntakeDriveCommand(
        //         m_IntakeSubsystem,
        //         () -> operatorController_HID.getLeftBumper(),
        //         () -> operatorController_HID.getRightBumper()
        //     )
        // );

        m_IntakeSubsystem.setDefaultCommand(
            new IntakeDriveCommand(
                m_IntakeSubsystem,
                () -> operatorController_HID.getLeftBumper(),
                () -> operatorController_HID.getRightBumper()
            )
        );

        m_VisionSubsystem.setDefaultCommand(new VisionCommand(m_VisionSubsystem));

        registerNamedCommands();

        autoChooser = new SendableChooser<Command>();
        configureAutoChooser();
        
        // Configure the trigger bindings
        configureBindings();
        initializeGameShuffleboard();

        
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

        // Auto Chooser Methods

    private void configureAutoChooser() {
        autoChooser.addOption("Simple Shoot", new ShootSpeaker(m_ShooterSubsystem, m_IntakeSubsystem));
        addOption("1 Note Stationary");
        addOption("1 Note Top");
        addOption("1 Note Mid");
        addOption("1 Note Bottom");
        addOption("2 Note Top");
        addOption("2 Note Mid");
        addOption("Forward Test Auto");
        addOption("Jank Test Auto");
    }

    private void addOption(String name) {
        autoChooser.addOption(name, autoManager.getAutoCommand(name));
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
        //     @Override
        //     public void execute() {
        //         m_PivotSubsystem.calibrate();
        //     }
        // }
        // );
        
        m_driverController.leftBumper().whileTrue(  
            new SlowModeSwerveCommand(
                m_SwerveSubsystem,
                () -> driverController_HID.getLeftY(),
                () -> driverController_HID.getLeftX(),
                () -> driverController_HID.getRightX(),
                () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband
            )
        );

        m_driverController.rightBumper().whileTrue(
            new TurboModeSwerveCommand(
                m_SwerveSubsystem,
                () -> driverController_HID.getLeftY(),
                () -> driverController_HID.getLeftX(),
                () -> driverController_HID.getRightX(),
                () -> driverController_HID.getRightTriggerAxis() > OIConstants.kDeadband
            )
        );

        m_driverController.b().onTrue(new InstantCommand() {
            @Override
            public void execute() {
                m_SwerveSubsystem.zeroHeading();
            }
        });

            // Drive Snapping Setup
       for(int angle = 0; angle < 360; angle += 45) {
        m_driverController.pov(angle).onTrue(
            new SnapSwerveCommand(
                m_SwerveSubsystem,
                () -> driverController_HID.getLeftY(),
                () -> driverController_HID.getLeftX(),
                () -> driverController_HID.getRightX(),
                angle
            )
        );
       }

       m_operatorController.y().whileTrue(
        new Command() {
            public void execute() {
                m_ShooterSubsystem.setShooterVelocity(m_ShooterSubsystem.getAmpRPM());
            }
        }
       );
       
        m_operatorController.povDown()
        // .or(m_operatorController.povUp())
        .onTrue(new InstantCommand() {
            @Override
            public void execute() {
                int index = m_PivotSubsystem.getIndex();
                m_ShooterSubsystem.setTargetRPM(ShooterConstants.PRESET_SPEEDS[index]);
            }
        });

        m_operatorController.x().onTrue(
            new IntakePivotCommand(m_IntakeSubsystem)
        );

        m_operatorController
        // .x()
        .povUp()
        .whileTrue(
            new SequentialCommandGroup(
                new IntakePivotCommand(m_IntakeSubsystem).onlyIf(() -> m_IntakeSubsystem.isUp()), //should we change this to .andThen() instead of sequential command group?
                new IntakeDriveAutoCommand(m_IntakeSubsystem)
            )
        )
        // .onFalse(
        //     new IntakePivotCommand(m_IntakeSubsystem).onlyIf(()-> !m_IntakeSubsystem.isUp())
        // );
        .whileFalse(
            // new SequentialCommandGroup(
                new IntakePivotCommand(m_IntakeSubsystem).onlyIf(()->!m_IntakeSubsystem.isUp())
                .alongWith(
                    new ResetPivotAutoCommand(m_PivotSubsystem).onlyIf(() -> !m_PivotSubsystem.isHandoffOk())
                )
                .andThen(
                    new HandoffAutoCommand(m_IntakeSubsystem, m_ShooterSubsystem)
                    .onlyIf(()-> m_PivotSubsystem.isHandoffOk() && m_IntakeSubsystem.isUp())
                )
            // )
        );

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
            }
        );

        m_operatorController.leftTrigger(OIConstants.kDeadband).whileTrue(
            new Command() {
                @Override
                public void execute() {
                    m_IntakeSubsystem.setIntake(IntakeConstants.SHOOTER_TRANSFER_SPEED);
                    // m_ShooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
                    m_ShooterSubsystem.setFeederSpeed(IntakeConstants.SHOOTER_TRANSFER_SPEED);
                }

                @Override
                public void end(boolean interrupted) {
                    m_IntakeSubsystem.intakeStop();
                    m_ShooterSubsystem.stopFeeder();
                }
            }
        );

        m_operatorController.start().onTrue(new InstantCommand() {
            @Override
            public void execute() {
                m_ClimbSubsystem.reset();
            }
        });
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ScoreSpeaker", new ParallelRaceGroup(new ShootSpeaker(m_ShooterSubsystem, m_IntakeSubsystem), new WaitCommand(5))); //.withTimeout(AutoConstants.SCORE_SPEAKER_TIMEOUT));
        NamedCommands.registerCommand("IntakeNote", CommandContainer.intakeNote(m_IntakeSubsystem));
        NamedCommands.registerCommand("ResetPivot", CommandContainer.enRoute(m_PivotSubsystem));
        NamedCommands.registerCommand("ToggleIntake", new IntakePivotCommand(m_IntakeSubsystem));
        NamedCommands.registerCommand("RevShooter", new RevShooterAutoCommand(m_ShooterSubsystem));
    }

    private void initializeGameShuffleboard() {
         ShuffleboardTab gameShuffleboardTab = TabManager
            .getInstance()
            .accessTab(SubsystemTab.GAME);

            // Generic Data - The heading, whether or not debug mode is active, and the auto chooser.
        gameShuffleboardTab.addNumber(  "Robot Heading",    () -> m_SwerveSubsystem.getHeading()).withWidget(BuiltInWidgets.kGyro).withPosition(10, 0).withSize(3, 4);
        gameShuffleboardTab.addBoolean( "Debug Mode",       () -> Constants.DEBUG_MODE_ACTIVE).withPosition(10, 4).withSize(3, 1);
        gameShuffleboardTab.add(        "Auto Chooser",     autoChooser).withPosition(8, 2).withSize(2, 1); 

            // Drive Layout - Shows which drive mode is active (Slow, Normal, Turbo)
        ShuffleboardLayout generalLayout = gameShuffleboardTab.getLayout("Drive Mode", BuiltInLayouts.kGrid).withPosition(5, 0).withProperties(Map.of("Number of columns", 4, "Number of Rows", 1)).withSize(5, 2);
        generalLayout.addBoolean(   "Slow Mode Active",    () -> DriveConstants.currentDriveMode == DriveMode.SLOW).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when false", "#808080", "Color when true", "#FFFF4"));
        generalLayout.addBoolean(   "Normal Mode Active",  () -> DriveConstants.currentDriveMode == DriveMode.NORMAL).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when false", "808080"));
        generalLayout.addBoolean(   "Turbo Mode Active",   () -> DriveConstants.currentDriveMode == DriveMode.TURBO).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when false", "#808080", "Color when true", "#0000FF"));
        generalLayout.addBoolean(   "Robot Centric Mode Active", () -> DriveConstants.isRobotCentric).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when false", "#808080", "Color when true", "#FF00FF"));

            // Intake Layout - Shows the deployed state of the intake and if the beam sensor is detecting something
        ShuffleboardLayout intakeLayout = gameShuffleboardTab.getLayout("Intake Data", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 2, "Number of Rows", 1)).withPosition(0, 0).withSize(5, 2);
        intakeLayout.addBoolean("Intake Down", () -> !m_IntakeSubsystem.isUp()).withWidget(BuiltInWidgets.kBooleanBox);
        intakeLayout.addBoolean("Beam Broken", () -> m_IntakeSubsystem.getBeamState()).withWidget(BuiltInWidgets.kBooleanBox);

            // Shooter Layout - Shows the Shooter RPM, if the Shooter has reached the target speed, and the position of the Shooter Pivot
        //ShuffleboardLayout shooterLayout = gameShuffleboardTab.getLayout("Shooter Data", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 4, "Number of Rows", 1)).withPosition(0, 4).withSize(6, 2);
        // shooterLayout.addNumber(    "Shooter RPM",      () -> m_ShooterSubsystem.getShooterVelocity()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Min", 0, "Max", 5000));
        // shooterLayout.addBoolean(   "Shooter At Speed", () -> m_ShooterSubsystem.isAtTarget()).withWidget(BuiltInWidgets.kBooleanBox);
        // shooterLayout.addBoolean("Line Break", () -> m_ShooterSubsystem.isProjectileFed()).withWidget(BuiltInWidgets.kBooleanBox);
        // shooterLayout.addNumber(    "Pivot Position",   () -> -m_PivotSubsystem.getDegrees()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Min", -60, "Max", 60));
        
        gameShuffleboardTab.addNumber(    "Shooter RPM", () -> m_ShooterSubsystem.getShooterVelocity()).withWidget(BuiltInWidgets.kGraph).withPosition(0, 3).withSize(3, 3);
        gameShuffleboardTab.addBoolean(    "Shooter At Speed", () -> m_ShooterSubsystem.getShooterVelocity() > 3900).withPosition(3, 3).withSize(2, 2);
        gameShuffleboardTab.addBoolean(    "Shooter Beam State", () -> m_ShooterSubsystem.isProjectileFed()).withPosition(3, 5).withSize(2, 1);

        //gameShuffleboardTab.add("Robot Pose", () -> m_SwerveSubsystem.getCurrentPose()).withWidget(BuiltInWidgets.kField).withPosition(4, 6);
        
        ShuffleboardLayout motorLayout = gameShuffleboardTab.getLayout("Motor Data", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 2, "Number of rows", 1)).withPosition(8, 3).withSize(2, 1);
        motorLayout.addNumber("Shooter Temp", () -> m_ShooterSubsystem.getShooterTemp()).withWidget(BuiltInWidgets.kTextView);
        motorLayout.addNumber("Intake Drive Temp", () -> m_IntakeSubsystem.getIntakeDriveTemp()).withWidget(BuiltInWidgets.kTextView);

            // Sets GAME to active tab
        //Shuffleboard.selectTab("GAME");
    }

  public Command getAutonomousCommand()  {
    // return m_SwerveSubsystem.navigateToPose(
    //   new Pose2d(2, 2, m_SwerveSubsystem.getRotation2d())
    // );
    //return autoChooser.getSelected();
    return autoChooser.getSelected();
  }

}