package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SimConstants;

/** Class unfinished */
public class IntakeIOSim implements IntakeIO {
    private DCMotor driveMotor, pivotMotor;
    private FlywheelSim driveSim;
    private SingleJointedArmSim pivotSim;
    private EncoderSim driveEncoder, pivotEncoder;

    private ProfiledPIDController pivotController;

    private boolean isDown;
    private double pivotTarget, driveTarget;

    // TODO: Drive in PID mode has not been added to simulation
    public IntakeIOSim() {
        throw new UnsupportedOperationException("Intake simulation is not yet supported");
        // driveMotor = DCMotor.getNEO(1);
        // pivotMotor = DCMotor.getNEO(2);

        // driveSim = new FlywheelSim(driveMotor, 1, SimConstants.INTAKE_DRIVE_MOI);
        // pivotSim = new SingleJointedArmSim(
        //     pivotMotor, 
        //     PivotConstants.GEAR_REDUCTION,
        //     SimConstants.INTAKE_PIVOT_MOI,
        //     SimConstants.INTAKE_PIVOT_ARM_LENGTH,
        //     IntakeConstants.PIVOT_MIN_ANGLE_RAD,
        //     IntakeConstants.PIVOT_MAX_ANGLE_RAD,
        //     true,
        //     IntakeConstants.PIVOT_MIN_ANGLE_RAD
        // );

        // pivotController = new ProfiledPIDController(
        //     IntakeConstants.kP_PIVOT,
        //     IntakeConstants.kI_PIVOT,
        //     IntakeConstants.kD_PIVOT,
        //     new TrapezoidProfile.Constraints(
        //         IntakeConstants.kMaxVelocity,
        //         IntakeConstants.kMaxAccel
        //     )
        // );

        // driveEncoder = EncoderSim.createForIndex(SimConstants.INTAKE_DRIVE_ENCODER_INDEX);
        // pivotEncoder = EncoderSim.createForIndex(SimConstants.INTAKE_PIVOT_ENCODER_INDEX);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: Update all inputs here
    }

    @Override
    public void setDriveRaw(double speed) {
        setDriveVoltage(getDriveVoltage(speed));
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSim.setInputVoltage(volts);
        updateDriveEncoderRate(volts);
    }

    @Override
    public void updateConstants(double kP, double kI, double kD, double maxSpeed, double maxAccel, double kP_Drive,
            double kI_Drive, double kD_Drive) {
        // TODO: Update all constants here
    }


    public double updateDriveEncoderRate(double voltage) {
        pivotEncoder.setRate(driveMotor.getSpeed(driveMotor.getTorque(driveSim.getCurrentDrawAmps()), voltage));
        return driveEncoder.getRate();
    }

    public double getDriveVoltage(double speed) {
        return driveMotor.nominalVoltageVolts * MathUtil.clamp(speed, -1, 1);
    }


    
}
