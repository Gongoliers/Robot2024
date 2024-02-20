package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotConstants;

public class PivotMotorIOTalonSRX implements PivotMotorIO {

    private final TalonSRX talonSRX;

    public PivotMotorIOTalonSRX() {
        int canId = 38;

        talonSRX = new TalonSRX(canId);
    }

    @Override
    public void configure() {
        boolean inverted = false;

        talonSRX.setInverted(inverted);
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
        talonSRX.set(TalonSRXControlMode.PercentOutput, volts / RobotConstants.BATTERY_VOLTAGE);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }
    
}
