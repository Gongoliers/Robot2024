package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.config.MechanismConfig;
import frc.robot.RobotConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** Simulated steer motor. */
public class SteerMotorIOSim implements SteerMotorIO {

  /** Represents the motor used to steer the wheel. */
  private final DCMotor motorSim = DCMotor.getFalcon500(1);

  /** Represents the wheel steered by the motor. */
  private final FlywheelSim wheelSim =
      new FlywheelSim(motorSim, MK4iConstants.STEER_GEARING, MK4iConstants.STEER_MOI);

  /** Represents the position of the steer motor. */
  private double positionRotations;

  /** Represents the velocity of the steer motor. */
  private double velocityRotationsPerSecond;

  /** PIDF position controller. */
  private final SteerMotorPIDF pidf;

  public SteerMotorIOSim() {
    positionRotations = 0.0;
    velocityRotationsPerSecond = 0.0;

    MechanismConfig pidfConstants = SwerveConstants.STEER_PIDF_CONSTANTS;

    // Simulation is an ideal environment that does not have friction
    pidfConstants =
        pidfConstants
            .withFeedforward(pidfConstants.feedforward.withStaticFeedforward(0.0))
            .withFeedback(pidfConstants.feedback.withPositionTolerance(0.0));

    pidf = new SteerMotorPIDF(pidfConstants);
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
    double voltage =
        pidf.calculate(
            Rotation2d.fromRotations(this.positionRotations),
            Rotation2d.fromRotations(positionRotations));

    wheelSim.setInputVoltage(voltage);
    wheelSim.update(RobotConstants.PERIODIC_DURATION);

    this.velocityRotationsPerSecond =
        Units.radiansToRotations(wheelSim.getAngularVelocityRadPerSec())
            / MK4iConstants.STEER_GEARING;
    this.positionRotations += this.velocityRotationsPerSecond;
  }
}
