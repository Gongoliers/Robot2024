package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  private double inputVoltage;

  /** Creates a new simulated elbow motor. */
  public WristMotorIOSim() {
    // TODO
    singleJointedArmSim =
        new SingleJointedArmSim(
            WristMotorConstants.JOINT_CONSTANTS.motor(),
            WristMotorConstants.JOINT_CONSTANTS.gearing(),
            ShoulderMotorConstants.JOINT_CONSTANTS.moiKgMetersSquared(),
            ShoulderMotorConstants.JOINT_CONSTANTS.lengthMeters() * 0.5,
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

    values.inputVoltage = inputVoltage;
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

    if (DriverStation.isEnabled()) {
      inputVoltage = feedbackVolts + feedforwardVolts;
      singleJointedArmSim.setInputVoltage(inputVoltage);
    } else {
      singleJointedArmSim.setInputVoltage(0.0);
    }
  }
}
