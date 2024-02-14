package frc.robot.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.ArmFeedforwardCalculator;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Simulated shoulder motor. */
public class ShoulderMotorIOSim implements ShoulderMotorIO {

  private final DCMotor motor;

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private final ArmFeedforward feedforward;

  private double appliedVolts;

  /** Creates a new simulated shoulder motor. */
  public ShoulderMotorIOSim() {
    motor = DCMotor.getNEO(1);

    singleJointedArmSim =
        new SingleJointedArmSim(
            motor,
            ShoulderMotorConstants.GEARING,
            ShoulderMotorConstants.MOI,
            ShoulderMotorConstants.SHOULDER_TO_ELBOW_DISTANCE,
            ShoulderMotorConstants.MINIMUM_ANGLE.getRadians(),
            ShoulderMotorConstants.MAXIMUM_ANGLE.getRadians(),
            true,
            0.0);

    feedback = new PIDController(ShoulderMotorConstants.KP, 0, 0);

    feedforward =
        new ArmFeedforward(
            0,
            ArmFeedforwardCalculator.calculateArmGravityCompensation(
                Rotation2d.fromDegrees(70.81), 0.101859),
            0);
  }

  @Override
  public void configure() {}

  @Override
  public void update(ShoulderMotorIOValues values) {
    singleJointedArmSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());
    values.appliedVolts = appliedVolts;
    values.currentAmps = singleJointedArmSim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double positionRotations) {
    singleJointedArmSim.setState(Units.rotationsToRadians(positionRotations), 0.0);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(measuredPositionRotations, velocityRotationsPerSecond);

    appliedVolts = feedbackVolts + feedforwardVolts;

    singleJointedArmSim.setInputVoltage(appliedVolts);
  }
}
