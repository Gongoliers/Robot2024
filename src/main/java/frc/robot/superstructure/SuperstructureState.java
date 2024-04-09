package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmConstants.ShoulderConstants;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeConstants.RollerConstants;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    ArmState armState,
    IntakeState intakeState,
    ShooterState shooterState) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(ShoulderConstants.INITIAL_ANGLE, 0, 0, 0);

  public static final SuperstructureState STOW =
      new SuperstructureState(ShoulderConstants.STOW_ANGLE, 0, 0, 0);

  public static final SuperstructureState INTAKE_POSITION =
      new SuperstructureState(ShoulderConstants.STOW_ANGLE, 0, 0, 0);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(
          ShoulderConstants.STOW_ANGLE,
          RollerConstants.INTAKE_VELOCITY,
          0,
          SerializerConstants.INTAKE_VELOCITY);

  public static final SuperstructureState PULL = new SuperstructureState(ShoulderConstants.STOW_ANGLE, 0, 0, SerializerConstants.PULL_VELOCITY);

  public static final SuperstructureState EJECT_POSITION = new SuperstructureState(ShoulderConstants.EJECT_ANGLE, 0, 0, 0);

  public static final SuperstructureState EJECT = new SuperstructureState(ShoulderConstants.EJECT_ANGLE, 0, 0, SerializerConstants.PULL_VELOCITY);

  public static final SuperstructureState SPEAKER_SHOOT =
      new SuperstructureState(
          ShoulderConstants.SHOOT_ANGLE,
          0,
          FlywheelConstants.SPEAKER_VELOCITY,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState PASS_SPIN =
      new SuperstructureState(
          ShoulderConstants.STOW_ANGLE, 0, FlywheelConstants.PASS_VELOCITY, 0);

  public static final SuperstructureState PASS_SHOOT =
      new SuperstructureState(
          ShoulderConstants.STOW_ANGLE,
          0,
          FlywheelConstants.PASS_VELOCITY,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState AMP_POSITION =
      new SuperstructureState(ShoulderConstants.AMP_ANGLE, 0, 0, 0);

  public static final SuperstructureState AMP_SPIN =
      new SuperstructureState(
          ShoulderConstants.AMP_ANGLE, 0, FlywheelConstants.AMP_VELOCITY, 0);

  public static final SuperstructureState AMP_SHOOT =
      new SuperstructureState(
          ShoulderConstants.AMP_ANGLE,
          0,
          FlywheelConstants.AMP_VELOCITY,
          SerializerConstants.SERIALIZE_VELOCITY);

  /**
   * Creates a new superstructure state.
   *
   * @param armState
   * @param intakeState 
   * @param shooterState 
   */
  public SuperstructureState {
    Objects.requireNonNull(armState);
    Objects.requireNonNull(intakeState);
    Objects.requireNonNull(shooterState);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param rollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState(
      Rotation2d shoulderAngle,
      double rollerVelocityRotationsPerSecond,
      double flywheelVelocityRotationsPerSecond,
      double serializerVelocityRotationsPerSecond) {
    this(
        new ArmState(shoulderAngle),
        new IntakeState(rollerVelocityRotationsPerSecond, rollerVelocityRotationsPerSecond),
        new ShooterState(flywheelVelocityRotationsPerSecond, serializerVelocityRotationsPerSecond));
  }

  /**
   * Returns true if at the superstructure goal.
   *
   * @param goal
   * @return true if at the superstructure goal.
   */
  public boolean atGoal(SuperstructureState goal) {
    return Arm.getInstance().atGoal()
        && Intake.getInstance().atGoal()
        && Shooter.getInstance().atGoal();
  }
}
