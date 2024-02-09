package frc.robot.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Simulated shoulder motor. */
public class ShoulderMotorIOSim implements ShoulderMotorIO {

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private final ArmFeedforward feedforward;

  /** Creates a new simulated shoulder motor. */
  public ShoulderMotorIOSim() {
    singleJointedArmSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            ShoulderMotorConstants.GEARING,
            0.1,
            ShoulderMotorConstants.SHOULDER_TO_ELBOW_DISTANCE,
            Units.degreesToRadians(10),
            Units.degreesToRadians(90),
            true,
            Units.degreesToRadians(90));

    feedback = new PIDController(10.0, 0, 0);

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

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  // TODO Remove, only for characterization
  @Override
  public void setVoltage(double volts) {
    System.out.println(volts);

    singleJointedArmSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
