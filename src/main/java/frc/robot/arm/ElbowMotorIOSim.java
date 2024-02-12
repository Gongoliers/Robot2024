package frc.robot.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.ArmFeedforwardCalculator;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;

/** Simulated elbow motor. */
public class ElbowMotorIOSim implements ElbowMotorIO {

  private final DCMotor motor = DCMotor.getNEO(1);

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private final ArmFeedforward feedforward;

  /** Creates a new simulated elbow motor. */
  public ElbowMotorIOSim() {
    singleJointedArmSim =
        new SingleJointedArmSim(
            motor,
            ElbowMotorConstants.GEARING,
            ElbowMotorConstants.MOI,
            ElbowMotorConstants.ELBOW_TO_WRIST_DISTANCE,
            ElbowMotorConstants.MINIMUM_ANGLE.getRadians(),
            ElbowMotorConstants.MAXIMUM_ANGLE.getRadians(),
            true,
            0.0);

    feedback = new PIDController(ElbowMotorConstants.KP, 0, 0);

    feedforward =
        new ArmFeedforward(
            0,
            ArmFeedforwardCalculator.calculateArmGravityCompensation(
                Rotation2d.fromDegrees(-54.873534), 0.152426),
            0);
  }

  @Override
  public void configure() {}

  @Override
  public void update(ElbowMotorIOValues values) {
    singleJointedArmSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());
    values.currentAmps = singleJointedArmSim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double positionRotations) {
    singleJointedArmSim.setState(Units.rotationsToRadians(positionRotations), 0);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(measuredPositionRotations, velocityRotationsPerSecond);

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  @Override
  public void setVoltage(double volts) {
    singleJointedArmSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
