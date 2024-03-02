package frc.robot.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.robot.climber.ClimberConstants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {

  private final CANSparkMax sparkMax;

  private final boolean isInverted;

  private static final double METERS_PER_ROTATION = 1.0;

  public ElevatorIOSparkMax(CAN can, boolean invert) {
    sparkMax = new CANSparkMax(can.id(), MotorType.kBrushless);

    isInverted = invert;
  }

  @Override
  public void configure() {
    sparkMax.setInverted(isInverted);

    Configurator.configureREV(() -> sparkMax.setIdleMode(IdleMode.kBrake));
  }

  @Override
  public void update(ElevatorIOValues values) {
    values.positionMeters = getPositionRotations() * METERS_PER_ROTATION;
  }

  /**
   * Gets the absolute position in rotations.
   *
   * @return the absolute position in rotations.
   */
  private double getPositionRotations() {
    return sparkMax.getEncoder().getPosition() / ElevatorConstants.GEARING;
  }

  @Override
  public void setPosition(double positionMeters) {
    double rotations = positionMeters / METERS_PER_ROTATION;

    sparkMax.getEncoder().setPosition(rotations);
  }

  @Override
  public void setVoltage(double volts) {
    sparkMax.setVoltage(volts);
  }
}
