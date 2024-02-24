package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.AccelerationCalculator;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import frc.robot.arm.ArmConstants.WristMotorConstants;

/** Simulated elbow motor. */
public class WristMotorIOSim implements WristMotorIO {

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private final AccelerationCalculator accelerationCalculator;

  private double appliedVolts;

  /** Creates a new simulated elbow motor. */
  public WristMotorIOSim() {
    // TODO
    singleJointedArmSim =
        new SingleJointedArmSim(
            ShoulderMotorConstants.JOINT_CONSTANTS.motor(),
            ShoulderMotorConstants.JOINT_CONSTANTS.gearing(),
            ShoulderMotorConstants.JOINT_CONSTANTS.moiKgMetersSquared(),
            ShoulderMotorConstants.JOINT_CONSTANTS.lengthMeters(),
            WristMotorConstants.MINIMUM_ANGLE.getRadians(),
            WristMotorConstants.MAXIMUM_ANGLE.getRadians(),
            false,
            0.0);

    feedback = new PIDController(WristMotorConstants.KP, 0, 0);

    accelerationCalculator = new AccelerationCalculator();
  }

  @Override
  public void configure() {}

  @Override
  public void update(WristMotorIOValues values) {
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
