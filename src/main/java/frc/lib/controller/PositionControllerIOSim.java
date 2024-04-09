package frc.lib.controller;

/** Simulated position controller. */
public class PositionControllerIOSim implements PositionControllerIO {

    private double positionRotations = 0.0;
    
    private double velocityRotationsPerSecond = 0.0;

    @Override
    public void configure(PositionControllerIOConstants constants) {}

    @Override
    public void update(PositionControllerIOValues values) {
        values.positionRotations = positionRotations;
        values.velocityRotationsPerSecond = velocityRotationsPerSecond;
    }

    @Override
    public void setPosition(double positionRotations) {
        this.positionRotations = positionRotations;
    }

    @Override
    public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
        this.positionRotations = positionRotations;
        this.velocityRotationsPerSecond = velocityRotationsPerSecond;
    }
    
}
