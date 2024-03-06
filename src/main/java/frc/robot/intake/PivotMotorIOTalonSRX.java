package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Configurator;
import frc.lib.SingleJointedArmFeedforward;
import frc.lib.SingleJointedArmFeedforward.SingleJointedArmFeedforwardConstants;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;

/** Pivot motor using a Talon SRX. */
public class PivotMotorIOTalonSRX implements PivotMotorIO {

  /** Hardware Talon SRX. */
  private final TalonSRX talonSRX;

  private final PIDController feedback;

  private final SingleJointedArmFeedforward feedforward;

  public PivotMotorIOTalonSRX() {
    talonSRX = new TalonSRX(8);

    feedback = new PIDController(16.0, 0, 0);

    SingleJointedArmFeedforwardConstants constants =
        new SingleJointedArmFeedforwardConstants().withKg(Rotation2d.fromDegrees((26)).plus(PivotMotorConstants.MASS_OFFSET), 1.8);

    feedforward = new SingleJointedArmFeedforward(constants);
  }

  @Override
  public void configure() {
    Configurator.configurePhoenix5(talonSRX::configFactoryDefault);

    talonSRX.setSensorPhase(true);

    Configurator.configurePhoenix5(
        () -> talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative));
  }

  @Override
  public void update(PivotMotorIOValues values) {
    values.positionRotations = getPivotPosition();
    values.velocityRotationsPerSecond = getPivotVelocity();
  }

  /**
   * Gets the position of the pivot in rotations.
   *
   * @return the position of the pivot in rotations.
   */
  private double getPivotPosition() {
    double rotations = talonSRX.getSelectedSensorPosition() / 2048.0;

    return rotations / PivotMotorConstants.SENSOR_GEARING;
  }

  /**
   * Setes the position of the pivot in rotations.
   *
   * @param positionRotations the position of the pivot in rotations.
   */
  private void setPivotPosition(double positionRotations) {
    double units = positionRotations * PivotMotorConstants.SENSOR_GEARING * 2048.0;

    Configurator.configurePhoenix5(() -> talonSRX.setSelectedSensorPosition(units));
  }

  private double getPivotVelocity() {
    double rotationsPerSecond = talonSRX.getSelectedSensorVelocity() / 2048.0;

    return rotationsPerSecond / PivotMotorConstants.SENSOR_GEARING;
  }

  @Override
  public void setPosition(double positionRotations) {
    setPivotPosition(positionRotations);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = getPivotPosition();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(
            Rotation2d.fromRotations(measuredPositionRotations)
                .plus(PivotMotorConstants.MASS_OFFSET));

    double percent = (feedbackVolts + feedforwardVolts) / talonSRX.getBusVoltage();

    talonSRX.set(TalonSRXControlMode.PercentOutput, percent);
  }
}
