package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;

/** Pivot motor using a Talon SRX. */
public class PivotMotorIOTalonSRX implements PivotMotorIO {

    /** Hardware Talon SRX. */
    private final TalonSRX talonSRX;

    public PivotMotorIOTalonSRX() {
        talonSRX = new TalonSRX(PivotMotorConstants.CAN.id());
    }

    @Override
    public void configure() {
        talonSRX.setInverted(PivotMotorConstants.IS_INVERTED);

        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    @Override
    public void update(PivotMotorIOValues values) {
        values.positionRotations = getPivotPosition();
    }

    /**
     * Gets the position of the pivot in rotations.
     * 
     * @return the position of the pivot in rotations.
     */
    private double getPivotPosition() {
        double rotations = talonSRX.getSelectedSensorPosition() / 2048;

        return rotations / PivotMotorConstants.SENSOR_GEARING;
    }

    /**
     * Setes the position of the pivot in rotations.
     * 
     * @param positionRotations the position of the pivot in rotations.
     */
    private void setPivotPosition(double positionRotations) {
        double units = positionRotations * PivotMotorConstants.SENSOR_GEARING * 2048;

        talonSRX.setSelectedSensorPosition(units);
    }

    @Override
    public void setPosition(double positionRotations) {
        setPivotPosition(positionRotations);
    }

    @Override
    public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
        // TODO
    }

    @Override
    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -PivotMotorConstants.MAXIMUM_VOLTAGE, PivotMotorConstants.MAXIMUM_VOLTAGE);

        double percent = volts / talonSRX.getBusVoltage();

        talonSRX.set(TalonSRXControlMode.PercentOutput, percent);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }
    
}
