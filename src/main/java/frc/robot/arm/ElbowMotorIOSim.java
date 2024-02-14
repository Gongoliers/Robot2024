package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.AccelerationCalculator;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;

/** Simulated elbow motor. */
public class ElbowMotorIOSim implements ElbowMotorIO {

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private final AccelerationCalculator accelerationCalculator;

  private double appliedVolts;

  /** Creates a new simulated elbow motor. */
  public ElbowMotorIOSim() {
    singleJointedArmSim =
        new SingleJointedArmSim(
            ElbowMotorConstants.JOINT_CONSTANTS.motor(),
            ElbowMotorConstants.JOINT_CONSTANTS.gearing(),
            ElbowMotorConstants.JOINT_CONSTANTS.moi(),
            ElbowMotorConstants.JOINT_CONSTANTS.length(),
            ElbowMotorConstants.MINIMUM_ANGLE.getRadians(),
            ElbowMotorConstants.MAXIMUM_ANGLE.getRadians(),
            false,
            0.0);

    feedback = new PIDController(ElbowMotorConstants.KP, 0, 0);

    accelerationCalculator = new AccelerationCalculator();
  }

  @Override
  public void configure() {}

  @Override
  public void update(ElbowMotorIOValues values) {
    singleJointedArmSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());
    values.velocityRotationsPerSecond =
        Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec());
    values.accelerationRotationsPerSecondPerSecond =
        accelerationCalculator.calculate(values.velocityRotationsPerSecond);

    values.appliedVolts = appliedVolts;
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

    double feedforwardVolts = 0.0;

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;

    singleJointedArmSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
