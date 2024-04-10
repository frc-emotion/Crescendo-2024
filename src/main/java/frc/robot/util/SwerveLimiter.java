package frc.robot.util;

import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveLimiter {
    private ChassisSpeedsRateLimiter rateLimiter;

    private TrapezoidProfile translationProfile, rotationProfile;

    private double maxSpeed, maxAngularSpeed;

    public SwerveLimiter(double maxSpeed, double maxAccel, double maxAngularSpeed, double maxAngularAccel) {
        rateLimiter = new ChassisSpeedsRateLimiter(maxAccel, maxAngularAccel);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
    }

    public SwerveLimiter(LimiterConstraints constraints) {
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

    /** Calculates the magnitude of the velocity vector from the ChassisSpeeds */
    private double getSpeed(ChassisSpeeds speeds) {
        return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    }

    /**
     * Limits the speeds of the ChassisSpeeds. If the current speed is greater than the max speed, then it multiplies the entire
     * ChassisSpeeds object by a multiplier equivalent to the amount above the max speed (ex. max is 1, currently at 1.2, multiplies by 0.83).
     * The method then checks whether the angular speed is above the max. If it is above the max, the angular speed is set to the max.
     * 
     * @param inputSpeeds The initial ChassisSpeeds object
     * @param speed The maximum translational speed
     * @param angularSpeed  The maximum angular sped
     * @return  The limited ChassisSpeeds object
     */
    private ChassisSpeeds limitSpeed(ChassisSpeeds inputSpeeds, double speed, double angularSpeed) {
        double multiplier = getSpeed(inputSpeeds) < speed ? 1 : speed / getSpeed(inputSpeeds);
        ChassisSpeeds output = inputSpeeds.times(multiplier);
        output.omegaRadiansPerSecond = inputSpeeds.omegaRadiansPerSecond < angularSpeed ? inputSpeeds.omegaRadiansPerSecond : angularSpeed;
        return output;
    }

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