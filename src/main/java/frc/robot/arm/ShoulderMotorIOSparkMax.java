package frc.robot.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.Configurator;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Shoulder motor using a Spark Max. */
public class ShoulderMotorIOSparkMax implements ShoulderMotorIO {

  /** Hardware Spark Max. */
  private final CANSparkMax sparkMax;

  /** Feedback controller for shoulder position. */
  private final PIDController feedback = new PIDController(24, 0, 0);

  // private final ProfiledPIDController feedback =
  //     new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

  /** Feedforward controller for shoulder position. */
  // private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

  /** Creates a new shoulder motor using a Spark Max. */
  public ShoulderMotorIOSparkMax() {
    sparkMax = new CANSparkMax(ShoulderMotorConstants.CAN.id(), MotorType.kBrushless);
  }

  @Override
  public void configure() {
    Configurator.configureREV(sparkMax::restoreFactoryDefaults);

    Configurator.configureREV(() -> sparkMax.setIdleMode(IdleMode.kBrake));
  }

  @Override
  public void update(ShoulderMotorIOValues values) {
    values.positionRotations = getPositionRotations();
    values.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setPosition(double positionRotations) {
    sparkMax.getEncoder().setPosition(positionRotations * ShoulderMotorConstants.GEARING);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    double measuredPositionRotations = getPositionRotations();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    // double feedforwardVolts =
    //     feedforward.calculate(measuredPositionRotations, feedback.getSetpoint().velocity);

    double kG = 0.1222 / Math.cos(Units.degreesToRadians(18.0));

    double feedforwardVolts = kG * Math.cos(Units.rotationsToRadians(measuredPositionRotations));

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  // TODO Remove, only for characterization
  @Override
  public void setVoltage(double volts) {
    if (volts > 0.0
        && getPositionRotations() > ShoulderMotorConstants.MAXIMUM_ANGLE.getRotations()) {
      volts = 0.0;
    } else if (volts < 0.0
        && getPositionRotations() < ShoulderMotorConstants.MINIMUM_ANGLE.getRotations()) {
      volts = 0.0;
    }

    sparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  /**
   * Gets the absolute position of the shoulder in rotations.
   *
   * @return the absolute position of the shoulder in rotations.
   */
  private double getPositionRotations() {
    return sparkMax.getEncoder().getPosition() / ShoulderMotorConstants.GEARING;
  }
}
