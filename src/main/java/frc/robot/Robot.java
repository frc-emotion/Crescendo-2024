// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand, m_ledCommand, m_controllerRumbleCommand;

    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        m_ledCommand = m_robotContainer.getLEDCommand();
        m_controllerRumbleCommand = m_robotContainer.getControllerRumbleCommand();

        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    public void disabledInit() {
        // Ensures that the LED Command is always running
        if (m_ledCommand != null && !CommandScheduler.getInstance().isScheduled(m_ledCommand)) {
            m_ledCommand.schedule();
        } else if (m_ledCommand == null) {
            DriverStation.reportWarning("LED Command not present (null)", false);
        }

        // Cancels the controller rumble Command when the robot is disabled
        if (m_controllerRumbleCommand != null
                && CommandScheduler.getInstance().isScheduled(m_controllerRumbleCommand)) {
            m_controllerRumbleCommand.cancel();
        }

        Shuffleboard.selectTab("AUTO");
    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Schedules the autonomous Command at the start of auto
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        } else {
            DriverStation.reportWarning("Autonomous Command not present (null)", false);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // Schedules the controller rumble Command at the beginning of teleop
        if (m_controllerRumbleCommand != null) {
            m_controllerRumbleCommand.schedule();
        } else {
            DriverStation.reportWarning("Controller Rumble Command not present (null)", false);
        }

        Shuffleboard.selectTab("GAME");
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
