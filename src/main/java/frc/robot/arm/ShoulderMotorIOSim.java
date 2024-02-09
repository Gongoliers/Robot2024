package frc.robot.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Simulated shoulder motor. */
public class ShoulderMotorIOSim implements ShoulderMotorIO {

  private final SingleJointedArmSim singleJointedArmSim;

  private final ProfiledPIDController feedback;

  private final ArmFeedforward feedforward;

  /** Creates a new simulated shoulder motor. */
  public ShoulderMotorIOSim() {
    singleJointedArmSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            ShoulderMotorConstants.GEARING,
            ShoulderMotorConstants.MOI,
            ShoulderMotorConstants.SHOULDER_TO_ELBOW_DISTANCE,
            ShoulderMotorConstants.MINIMUM_ANGLE.getRadians(),
            ShoulderMotorConstants.MAXIMUM_ANGLE.getRadians(),
            true,
            Units.degreesToRadians(90));

    feedback =
        new ProfiledPIDController(
            ShoulderMotorConstants.KP,
            0,
            0,
            new Constraints(
                ShoulderMotorConstants.MAXIMUM_SPEED, ShoulderMotorConstants.MAXIMUM_ACCELERATION));

    double kG = 0.101859 / Math.cos(Units.degreesToRadians(70.81));

    feedforward = new ArmFeedforward(0, kG, 0);
  }

  @Override
  public void configure() {}

  @Override
  public void update(ShoulderMotorIOValues values) {
    singleJointedArmSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());
    values.currentAmps = singleJointedArmSim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double positionRotations) {
    singleJointedArmSim.setState(Units.rotationsToRadians(positionRotations), 0.0);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    double measuredPositionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts = feedforward.calculate(measuredPositionRotations, 0.0);

    singleJointedArmSim.setInputVoltage(feedbackVolts + feedforwardVolts);
  }
}
