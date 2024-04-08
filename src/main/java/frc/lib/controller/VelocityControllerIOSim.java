package frc.lib.controller;

/** Simulated velocity controller. */
public class VelocityControllerIOSim implements VelocityControllerIO {

    private double velocityRotationsPerSecond = 0.0;

    @Override
    public void configure(VelocityControllerIOConstants constants) {}

    @Override
    public void update(VelocityControllerIOValues values) {
        values.velocityRotationsPerSecond = velocityRotationsPerSecond;
    }

    @Override
    public void setSetpoint(double velocityRotationsPerSecond) {
        this.velocityRotationsPerSecond = velocityRotationsPerSecond;
    }
    
}
