package frc.robot.util;

public class PeriodicFunction {
    public Runnable runnable;
    public double period_ms;

    public PeriodicFunction(Runnable runnable, double period_ms) {
        this.runnable = runnable;
        this.period_ms = period_ms;
    }
}
