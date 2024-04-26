package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Subsystem;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MotionProfileConfig;
import frc.lib.config.MotorConfig;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;

/** Intake subsystem. */
public class Intake extends Subsystem {

  /** Intake singleton. */
  private static Intake instance = null;

  /** Front roller config. */
  private final MechanismConfig frontRollerConfig =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withNeutralBrake(false)
                  .withMotorToMechanismRatio(24.0 / 16.0))
          .withMotionProfileConfig(
              new MotionProfileConfig().withMaximumVelocity(66) // rotations per second
              )
          .withFeedforwardConfig(
              new FeedforwardControllerConfig()
                  .withStaticFeedforward(0.13) // volts
                  .withVelocityFeedforward(0.1683) // volts per rotation per second
              )
          .withFeedbackConfig(
              new FeedbackControllerConfig()
                  .withProportionalGain(0.1) // volts per rotation per second
              );

  /** Back roller config. */
  private final MechanismConfig backRollerConfig =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withNeutralBrake(false)
                  .withMotorToMechanismRatio(24.0 / 16.0))
          .withMotionProfileConfig(
              new MotionProfileConfig().withMaximumVelocity(66) // rotations per second
              )
          .withFeedforwardConfig(
              new FeedforwardControllerConfig()
                  .withStaticFeedforward(0.13) // volts
                  .withVelocityFeedforward(0.1759) // volts per rotation per second
              )
          .withFeedbackConfig(
              new FeedbackControllerConfig()
                  .withProportionalGain(0.1) // volts per rotation per second
              );

  /** Rollers. */
  private final VelocityControllerIO frontRoller, backRoller;

  /** Roller values. */
  private final VelocityControllerIOValues frontRollerValues, backRollerValues;

  /** Intake goal. Set by superstructure. */
  private IntakeState goal;

  /** Intake setpoint. Updated periodically to reach goal within constraints. */
  private IntakeState setpoint;

  /** Initializes the intake subsystem and configures intake hardware. */
  private Intake() {
    frontRoller = IntakeFactory.createFrontRoller(frontRollerConfig);
    frontRoller.configure();

    frontRollerValues = new VelocityControllerIOValues();

    backRoller = IntakeFactory.createBackRoller(backRollerConfig);
    backRoller.configure();

    backRollerValues = new VelocityControllerIOValues();

    setpoint = IntakeState.IDLING;
    goal = IntakeState.IDLING;
  }

  /**
   * Returns the intake subsystem instance.
   *
   * @return the intake subsystem instance.
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  @Override
  public void periodic() {
    frontRoller.update(frontRollerValues);
    backRoller.update(backRollerValues);

    setpoint = goal;

    frontRoller.setSetpoint(setpoint.frontRollerVelocityRotationsPerSecond());
    backRoller.setSetpoint(setpoint.backRollerVelocityRotationsPerSecond());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Front Roller", frontRollerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Back Roller", backRollerValues);
  }

  /**
   * Returns the intake state.
   *
   * @return the intake state.
   */
  public IntakeState getState() {
    return new IntakeState(
        frontRollerValues.velocityRotationsPerSecond, backRollerValues.velocityRotationsPerSecond);
  }

  /**
   * Sets the intake goal state.
   *
   * @param goal the intake goal state.
   */
  public void setGoal(IntakeState goal) {
    this.goal = goal;
  }

  /**
   * Returns true if the intake is at the goal state.
   *
   * @return true if the intake is at the goal state.
   */
  public boolean atGoal() {
    return getState().at(goal);
  }

  /**
   * Returns a trigger for if a note is stuck.
   *
   * @return a trigger for if a note is stuck.
   */
  public Trigger noteStuck() {
    return new Trigger(() -> frontRollerStuck() || backRollerStuck());
  }

  /**
   * Returns true if the front roller is stuck.
   *
   * @return true if the front roller is stuck.
   */
  private boolean frontRollerStuck() {
    return frontRollerValues.motorAmps > 40.0;
  }

  /**
   * Returns true if the back roller is stuck.
   *
   * @return true if the back roller is stuck.
   */
  private boolean backRollerStuck() {
    return backRollerValues.motorAmps > 40.0;
  }
}
