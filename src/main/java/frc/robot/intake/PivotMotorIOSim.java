package frc.robot.intake;

public class PivotMotorIOSim  implements PivotMotorIO {

    @Override
    public void configure() {
    }

    @Override
    public void update(PivotMotorIOValues values) {
        // TODO
        values.positionRotations = 0.0;
    }

    @Override
    public void setPosition(double positionRotations) {
        // TODO
    }

    @Override
    public void setSetpoint(double positionRotations) {
        // TODO
    }

    @Override
    public void setVoltage(double volts) {
        // TODO
    }

    @Override
    public void stop() {
        // TODO
    }
    
}
