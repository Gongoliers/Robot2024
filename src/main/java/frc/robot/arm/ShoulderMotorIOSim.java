package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.AccelerationCalculator;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Simulated shoulder motor. */
public class ShoulderMotorIOSim implements ShoulderMotorIO {

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private double inputVoltage;

  private final AccelerationCalculator accelerationCalculator;

  /** Creates a new simulated shoulder motor. */
  public ShoulderMotorIOSim() {
    singleJointedArmSim =
        new SingleJointedArmSim(
            ShoulderMotorConstants.JOINT_CONSTANTS.motor(),
            ShoulderMotorConstants.JOINT_CONSTANTS.gearing(),
            ShoulderMotorConstants.JOINT_CONSTANTS.moiKgMetersSquared(),
            ShoulderMotorConstants.JOINT_CONSTANTS.lengthMeters(),
            ShoulderMotorConstants.MINIMUM_ANGLE.getRadians(),
            ShoulderMotorConstants.MAXIMUM_ANGLE.getRadians(),
            false,
            0.0);

    feedback = new PIDController(ShoulderMotorConstants.KP, 0, 0);

    accelerationCalculator = new AccelerationCalculator();
  }

  @Override
  public void configure() {}

  @Override
  public void update(ShoulderMotorIOValues values) {
    singleJointedArmSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionRotations = getPosition();
    values.velocityRotationsPerSecond = getVelocity();
    values.accelerationRotationsPerSecondPerSecond = getAcceleration();

    values.inputVoltage = inputVoltage;
    values.currentAmps = singleJointedArmSim.getCurrentDrawAmps();
  }

  private double getPosition() {
    return Units.radiansToRotations(singleJointedArmSim.getAngleRads());
  }

  private double getVelocity() {
    return Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec());
  }

  private double getAcceleration() {
    return accelerationCalculator.calculate(getVelocity());
  }

  @Override
  public void setPosition(double positionRotations) {
    singleJointedArmSim.setState(Units.rotationsToRadians(positionRotations), 0.0);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = getPosition();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts = 0.0;

    if (DriverStation.isEnabled()) {
      inputVoltage = feedbackVolts + feedforwardVolts;
      singleJointedArmSim.setInputVoltage(inputVoltage);
    } else {
      singleJointedArmSim.setInputVoltage(0.0);
    }
  }
}
