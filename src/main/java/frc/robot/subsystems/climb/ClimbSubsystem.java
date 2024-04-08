package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.RobotDataMode;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Climb Subsystem
 */
public class ClimbSubsystem extends SubsystemBase {
    SendableNumber ClimbMax = new SendableNumber(SubsystemTab.CLIMB, "Climb Extension Limit", ClimbConstants.EXTENSION_LIMIT);

    private ClimbIO io;
    private final ClimbIOInputs inputs = new ClimbIOInputs();

    /**
     * Constructs a ClimbSubsystem instance
     */
    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
        initShuffleboard();
    }

    public ClimbSubsystem() {
        this(new ClimbIOSparkMax());
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    /**
     * Initialize Climb Shuffleboard Information
     */
    private void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == RobotDataMode.MATCH) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.CLIMB);
        ShuffleboardLayout persianPositions = moduleData.getLayout(
            "Persian Positions",
            BuiltInLayouts.kList
        );

        persianPositions.addNumber(
            "Left Climb Position",
            () -> getLeftPos()
        );

        persianPositions.addNumber(
            "Right Climb Position",
            () -> getRightPos()
        );

        persianPositions.addNumber(
            "Average Climb Position",
            () -> getPosition()    
        );

        persianPositions.addBoolean(
            "Is Climb Aligned",
            () -> isClimbAligned()
        );

        persianPositions.withSize(2, 4);
    }

    /**
     * Set speed of climb motors
     * @param speed Speed to set climb motors to 0 to 1
     */
    public void setRawSpeed(double speed) {
        io.set(speed);
    }

    /** Returns the current highest position of the climb */
    public double getPosition() {
        return inputs.highestPos;
    }

    public double getLeftPos() {
        return inputs.leftPos;
    }

    public double getRightPos() {
        return inputs.rightPos;
    }

    public boolean isClimbAligned() {
        return inputs.isClimbAligned;
    }

    public void stop() {
        io.stop();
    }

    public double getClimbMaxExtension() {
        return ClimbMax.get();
    }
}
