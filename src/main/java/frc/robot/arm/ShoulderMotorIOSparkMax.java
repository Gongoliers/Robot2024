package frc.robot.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.Configurator;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Shoulder motor using a Spark Max. */
public class ShoulderMotorIOSparkMax implements ShoulderMotorIO {

  /** Hardware Spark Max. */
  private final CANSparkMax sparkMax;

  /** Feedback controller for shoulder position. */
  private final ProfiledPIDController feedback;

  /** Feedforward controller for shoulder position. */
  // private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

  /** Creates a new shoulder motor using a Spark Max. */
  public ShoulderMotorIOSparkMax() {
    sparkMax = new CANSparkMax(ShoulderMotorConstants.CAN.id(), MotorType.kBrushless);

    feedback =
        new ProfiledPIDController(
            ShoulderMotorConstants.KP,
            0,
            0,
            new Constraints(
                ShoulderMotorConstants.MAXIMUM_SPEED, ShoulderMotorConstants.MAXIMUM_ACCELERATION));
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

    sparkMax.setVoltage(feedbackVolts + feedforwardVolts);
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
