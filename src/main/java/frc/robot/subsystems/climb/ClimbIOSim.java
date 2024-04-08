package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSim implements ClimbIO {
    private ElevatorSim sim;

    @SuppressWarnings("unused")
    public ClimbIOSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(2),
            ClimbConstants.CLIMB_WEIGHT,
            ClimbConstants.GEAR_REDUCTION,
            ClimbConstants.SPOOL_RADIUS,
            ClimbConstants.MIN_HEIGHT_METERS,
            ClimbConstants.MAX_HEIGHT_METERS,
            true,
            ClimbConstants.MIN_HEIGHT_METERS
        );


        
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.leftPos = sim.getPositionMeters();
        inputs.rightPos = sim.getPositionMeters();
    }

    @Override
    public void set(double speed) {
        sim.setInputVoltage(MathUtil.clamp(speed, -1, 1) * 12.0);
    }
}
