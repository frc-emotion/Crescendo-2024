// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SwerveXboxCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.ShooterManualCommand;
import frc.robot.commands.SwerveXboxCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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

  public static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_SwerveSubsystem.setDefaultCommand(
    new SwerveXboxCommand(
      m_SwerveSubsystem,
      () -> -m_driverController.getRawAxis(OIConstants.kDriverYAxis),
      () -> -m_driverController.getRawAxis(OIConstants.kDriverXAxis),
      () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !m_driverController.a().getAsBoolean(),
      () -> m_driverController.leftBumper().getAsBoolean(),
      () -> m_driverController.rightBumper().getAsBoolean(),
      () -> m_driverController.getRightTriggerAxis(),
      () -> m_driverController.getLeftTriggerAxis()
    ));

    // FIX
    // m_IntakeSubsystem.setDefaultCommand(new RunIntake(m_IntakeSubsystem, () -> m_operatorController.rightBumper.getAsBoolean(), () -> m_operatorController.leftBumper.getAsBoolean()));
        
    // Configure the trigger bindings
    configureBindings();
  }

  // The robot's subsystems and commands are defined here...
    public static final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
    public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
        OIConstants.kDriverControllerPort
    );
    private final CommandXboxController m_operatorController = new CommandXboxController(
        OIConstants.kOperatorControllerPort
    );

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_SwerveSubsystem.setDefaultCommand(
            new SwerveXboxCommand(
                m_SwerveSubsystem,
                () -> -m_driverController.getRawAxis(OIConstants.kDriverYAxis),
                () -> -m_driverController.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_driverController.a().getAsBoolean(),
                () -> m_driverController.leftBumper().getAsBoolean(),
                () -> m_driverController.rightBumper().getAsBoolean(),
                () -> m_driverController.getRightTriggerAxis(),
                () -> m_driverController.getLeftTriggerAxis()
            )
        );

        // m_shooterSubsystem.setDefaultCommand(
        //     new ShooterManualCommand(
        //         () -> m_operatorController.leftBumper().getAsBoolean(),
        //         () -> m_operatorController.getLeftTriggerAxis(),
        //         m_shooterSubsystem
        //     )
        // );
        // Configure the trigger bindings
        configureBindings();
    }

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    new JoystickButton(m_operatorController.getHID(), XboxController.Button.kB.value).onTrue(new PivotIntake(m_IntakeSubsystem));
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
    }

    public Command getAutonomousCommand() {
        throw new UnsupportedOperationException(
            "Unimplemented method 'getAutonomousCommand'"
        );
    }
}
