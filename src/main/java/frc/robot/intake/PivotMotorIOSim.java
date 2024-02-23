package frc.robot.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.SingleJointedArmFeedforward;
import frc.robot.RobotConstants;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;

/** Simulated pivot motor. */
public class PivotMotorIOSim implements PivotMotorIO {

  private final DCMotor motor;

  private final SingleJointedArmSim singleJointedArmSim;

  private final PIDController feedback;

  private final SingleJointedArmFeedforward feedforward;

  public PivotMotorIOSim() {
    motor = DCMotor.getVex775Pro(1);

    singleJointedArmSim =
        new SingleJointedArmSim(
            motor,
            PivotMotorConstants.MOTOR_GEARING,
            PivotMotorConstants.MOI,
            PivotMotorConstants.DISTANCE,
            PivotMotorConstants.MINIMUM_ANGLE.getRadians(),
            PivotMotorConstants.MAXIMUM_ANGLE.getRadians(),
            true,
            0.0);

    feedback = new PIDController(PivotMotorConstants.KP, 0, 0);

    feedforward = new SingleJointedArmFeedforward(0, 0, 0);
  }

  @Override
  public void configure() {}

  @Override
  public void update(PivotMotorIOValues values) {
    singleJointedArmSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionRotations = Units.radiansToRotations(singleJointedArmSim.getAngleRads());
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
        feedforward.calculate(
            Rotation2d.fromRotations(measuredPositionRotations), velocityRotationsPerSecond);

    singleJointedArmSim.setInputVoltage(feedbackVolts + feedforwardVolts);
  }

  @Override
  public void setVoltage(double volts) {
    singleJointedArmSim.setInputVoltage(0);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
