package frc.robot.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Simulated shoulder motor. */
public class ShoulderMotorIOSim implements ShoulderMotorIO {

  private final SingleJointedArmSim singleJointedArmSim;

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
            false,
            Units.degreesToRadians(90));
  }

  @Override
  public void configure() {
  }

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
  public void runSetpoint(double positionRotations) {
    //setPosition(positionRotations);
  }

  // TODO Remove, only for characterization
  @Override
  public void setVoltage(double volts) {
    singleJointedArmSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
