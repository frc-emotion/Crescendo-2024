package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.console.SimConsoleSource;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.SimConstants;

public class ClimbIOSim implements ClimbIO {
    private ElevatorSim sim;
    private DCMotor motor;
    private EncoderSim encoderSim;

    @SuppressWarnings("unused")
    public ClimbIOSim() {
        motor = DCMotor.getNEO(2);
        sim = new ElevatorSim(
            motor,
            ClimbConstants.CLIMB_WEIGHT,
            ClimbConstants.GEAR_REDUCTION,
            ClimbConstants.SPOOL_RADIUS,
            ClimbConstants.MIN_HEIGHT_METERS,
            ClimbConstants.MAX_HEIGHT_METERS,
            false,
            ClimbConstants.MIN_HEIGHT_METERS
        );

        encoderSim = EncoderSim.createForIndex(SimConstants.CLIMB_ENCODER_INDEX);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.leftPos = getPosition();
        inputs.rightPos = getPosition();
        inputs.highestPos = getPosition();
        inputs.isClimbAligned = true;
        inputs.current = sim.getCurrentDrawAmps();
    }

    @Override
    public void set(double speed) {
        double voltage = MathUtil.clamp(speed, -1, 1) * 12.0;
        sim.setInputVoltage(voltage);
        encoderSim.setRate(motor.getSpeed(motor.getTorque(sim.getCurrentDrawAmps()) * ClimbConstants.GEAR_REDUCTION, voltage));
    }

    @Override
    public void reset() {
        encoderSim.setCount(0);
    }

    public double getPosition() {
        return encoderSim.getDistance();
    }
}
