package frc.robot.util;

import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** 
 * Limits the maximum speed and acceleration of given {@link ChassisSpeeds}. Uses the
 * PathPlanner {@link ChassisSpeedsRateLimiter} for acceleration limiting.
 */
public class SwerveLimiter {
    private ChassisSpeedsRateLimiter rateLimiter;

    private double maxSpeed, maxAngularSpeed;

    public SwerveLimiter(LimiterConstraints constraints) {
        this.rateLimiter = new ChassisSpeedsRateLimiter(constraints.maxAcceleration, constraints.maxAngularAcceleration);
        this.maxSpeed = constraints.maxSpeed;
        this.maxAngularSpeed = constraints.maxAngularSpeed;

        setConstraints(constraints);
    }


    /** Calculates and limits the ChassisSpeeds */
    public ChassisSpeeds calculate(ChassisSpeeds input) {
        ChassisSpeeds output = rateLimiter.calculate(input);
        return limitSpeed(output, maxSpeed, maxAngularSpeed);
    }

    /** Sets the constraints */
    public void setConstraints(double maxAccel, double maxAngularAccel, double maxSpeed, double maxAngularSpeed) {
        rateLimiter.setRateLimits(maxAccel, maxAngularAccel);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
    }

    /** Sets the constraints using a LimiterConstraints object */
    public void setConstraints(LimiterConstraints constraints) {
        rateLimiter.setRateLimits(constraints.maxAcceleration, constraints.maxAngularAcceleration);
        this.maxSpeed = constraints.maxSpeed;
        this.maxAngularSpeed = constraints.maxAngularSpeed;
    }

    /** Calculates the magnitude of the velocity vector from the given ChassisSpeeds */
    private double getSpeed(ChassisSpeeds speeds) {
        return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    }

    /**
     * Limits the speed of the ChassisSpeeds to the maximum speed provided. If the speed of the robot, as given by 
     * the {@link #getSpeed()} method calculates the current
     * angle the robot is facing. It then takes the sine/cosine of the current angle and multiplies it by the max
     * speed in order to find the appropriate speed for the current angle. Finally, it limits the angular speed by checking
     * if it is higher than the maximum angular speed.
     * 
     * @param inputSpeeds The initial ChassisSpeeds object
     * @param speed The maximum translational speed
     * @param angularSpeed  The maximum angular sped
     * @return  The limited {@link ChassisSpeeds} object
     * @see <a href="https://www.desmos.com/calculator/scjw7vizwa">Desmos Visualizer</a>
     */
    private ChassisSpeeds limitSpeed(ChassisSpeeds inputSpeeds, double maxTranslationalSpeed, double maxAngularSpeed) {
        ChassisSpeeds outputSpeeds;
        if(getSpeed(inputSpeeds) > maxTranslationalSpeed) {
                // Calculates the angle which the robot is currently travelling at
            double angle = Math.atan(inputSpeeds.vyMetersPerSecond / inputSpeeds.vxMetersPerSecond);

                // If the current y velocity is less than zero, the angle is flipped 180 degrees
            if(inputSpeeds.vyMetersPerSecond < 0) {
                angle += Math.PI;
            }

                // Calculates then sets the new speeds
            outputSpeeds = new ChassisSpeeds(
                maxTranslationalSpeed * Math.sin(angle),
                maxTranslationalSpeed * Math.cos(angle),
                0
            );
        } else {
            outputSpeeds = new ChassisSpeeds(
                inputSpeeds.vxMetersPerSecond,
                inputSpeeds.vyMetersPerSecond,
                0
            );
        }

        if(inputSpeeds.omegaRadiansPerSecond > maxAngularSpeed) {
            outputSpeeds.omegaRadiansPerSecond = maxAngularSpeed;
        } else {
            outputSpeeds.omegaRadiansPerSecond = inputSpeeds.omegaRadiansPerSecond;
        }

        return outputSpeeds;
    }

    /**
     * Provides acceleration and velocity constraints to the {@link SwerveLimiter} class.
     */
    public static class LimiterConstraints {
        public double maxAcceleration, maxAngularAcceleration, maxSpeed, maxAngularSpeed;

        public LimiterConstraints(double maxSpeed, double maxAccel, double maxAngularSpeed, double maxAngularAccel) {
            this.maxAcceleration = maxAccel;
            this.maxAngularAcceleration = maxAngularAccel;
            this.maxSpeed = maxSpeed;
            this.maxAngularSpeed = maxAngularSpeed;
        }
        
        /** Multiplies all the constants by a scalar value */
        public LimiterConstraints mult(double multiplier) {
            return new LimiterConstraints(
                maxAcceleration * multiplier,
                maxAngularAcceleration * multiplier,
                maxSpeed * multiplier,
                maxAngularSpeed * multiplier
            );
        }
    }
}