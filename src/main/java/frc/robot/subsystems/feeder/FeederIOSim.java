package frc.robot.subsystems.feeder;

import javax.swing.plaf.basic.BasicTextAreaUI;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimConstants;

public class FeederIOSim implements FeederIO {
    private DCMotor feederMotor;
    private FlywheelSim sim;
    private EncoderSim encoder;

    private PIDController controller;

    private double targetVelocity;

    public FeederIOSim() {
        feederMotor = DCMotor.getNEO(1);
        sim = new FlywheelSim(feederMotor, 1, SimConstants.FEEDER_MOI);
        encoder = EncoderSim.createForIndex(SimConstants.FEEDER_ENCODER_INDEX);

        controller = new PIDController(ShooterConstants.FEEDER_kP, ShooterConstants.FEEDER_kI, ShooterConstants.FEEDER_kD);
        targetVelocity = 0;
    }

    @Override
    public void updateInputs(FeederIOInputsAutoLogged inputs) {
        inputs.isAtTarget = isAtTarget();
        inputs.targetVelocity = targetVelocity;
        inputs.velocity = encoder.getRate();
        inputs.current = sim.getCurrentDrawAmps();
    }

    @Override
    public void updateConstants(double kP, double kI, double kD, double kFF) {
        controller.setPID(kP, kI, kD);
    }

    @Override
    public void set(double speed) {
        setVoltage(feederMotor.nominalVoltageVolts * MathUtil.clamp(speed, -1, 1));
    }
    
    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
        updateEncoderRate(voltage);
    }

    @Override
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        double voltage = controller.calculate(encoder.getRate(), velocity) * 12.0;
        sim.setInputVoltage(voltage);
        updateEncoderRate(voltage);
    }

    /** Calculates and sets the expected rate of the motor in radians per second */
    public void updateEncoderRate(double voltage) {
        encoder.setRate(feederMotor.getSpeed(feederMotor.getTorque(sim.getCurrentDrawAmps()), voltage));
    }

    public boolean isAtTarget() {
        return controller.atSetpoint();
    }
}
