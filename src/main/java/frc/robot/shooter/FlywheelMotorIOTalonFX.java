package frc.robot.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelMotorIOTalonFX implements FlywheelMotorIO {

    private final TalonFX talonFX;

    public FlywheelMotorIOTalonFX() {
        int canId = 39;

        talonFX = new TalonFX(canId);
    }


    @Override
    public void configure() {
        talonFX.setInverted(false);

        talonFX.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void update(FlywheelMotorIOValues values) {
        values.angularVelocityRotationsPerSecond = talonFX.getVelocity().getValue(); // TODO convert?
        values.currentAmps = talonFX.getStatorCurrent().getValue();
    }

    @Override
    public void setVoltage(double volts) {
        talonFX.setVoltage(volts);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }
    
}
