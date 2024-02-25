package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.SingleJointedArmFeedforward;
import frc.lib.ArmFeedforwardCalculator;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;

/** Pivot motor using a Talon SRX. */
public class PivotMotorIOTalonSRX implements PivotMotorIO {

  /** Hardware Talon SRX. */
  private final TalonSRX talonSRX;

  private final PIDController feedback;

  private final SingleJointedArmFeedforward feedforward;

  public PivotMotorIOTalonSRX() {
    talonSRX = new TalonSRX(PivotMotorConstants.CAN.id());

    feedback = new PIDController(PivotMotorConstants.KP, 0, 0);

    // TODO
    double kg = ArmFeedforwardCalculator.calculateArmGravityCompensation(Rotation2d.fromDegrees(26), 1.8);

    feedforward = new SingleJointedArmFeedforward(0, kg, 0);
  }

  @Override
  public void configure() {
    talonSRX.configFactoryDefault();

    talonSRX.setSensorPhase(PivotMotorConstants.IS_SENSOR_INVERTED);
    talonSRX.setInverted(PivotMotorConstants.IS_MOTOR_INVERTED);

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
    double rotations = talonSRX.getSelectedSensorPosition() / 2048.0;

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
    double measuredPositionRotations = getPivotPosition();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(
            Rotation2d.fromRotations(measuredPositionRotations), velocityRotationsPerSecond);

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  @Override
  public void setVoltage(double volts) {
    double percent = volts / talonSRX.getBusVoltage();

    talonSRX.set(TalonSRXControlMode.PercentOutput, percent);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
