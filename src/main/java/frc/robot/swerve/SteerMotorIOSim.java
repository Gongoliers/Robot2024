package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;
import frc.robot.swerve.SwerveConstants.SteerMotorConstants;

/** Simulated steer motor. */
public class SteerMotorIOSim implements SteerMotorIO {

  /** Represents the motor used to steer the wheel. */
  private final DCMotor motorSim = DCMotor.getFalcon500(1); // TODO

  /** Represents the wheel steered by the motor. */
  private final FlywheelSim wheelSim =
      new FlywheelSim(motorSim, MK4iConstants.STEER_GEARING, MK4iConstants.STEER_MOI);

  /** Represents the position of the steer motor. */
  private double positionRotations;

  /** Represents the velocity of the steer motor. */
  private double velocityRotationsPerSecond;

  /** Feedback controller for the position. */
  private final PIDController positionFeedback =
      new PIDController(SteerMotorConstants.FEEDBACK_KP, 0, SteerMotorConstants.FEEDBACK_KD);

  public SteerMotorIOSim() {
    positionRotations = 0.0;
    velocityRotationsPerSecond = 0.0;

    positionFeedback.enableContinuousInput(0, 1);
    positionFeedback.setTolerance(SteerMotorConstants.TOLERANCE.getRotations());
  }

  @Override
  public void configure() {}

  @Override
  public void update(SteerMotorIOValues values) {
    values.positionRotations = positionRotations;
    values.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }

  @Override
  public void setPosition(double positionRotations) {
    this.positionRotations = positionRotations;
  }

  @Override
  public void setSetpoint(double positionRotations) {
    double voltage = positionFeedback.calculate(this.positionRotations, positionRotations);

    wheelSim.setInputVoltage(voltage);
    wheelSim.update(RobotConstants.PERIODIC_DURATION);

    this.velocityRotationsPerSecond =
        Units.radiansToRotations(wheelSim.getAngularVelocityRadPerSec())
            / MK4iConstants.STEER_GEARING;
    this.positionRotations += this.velocityRotationsPerSecond;
  }
}
