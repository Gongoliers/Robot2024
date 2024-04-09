package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;
import frc.robot.intake.IntakeConstants.BackRollerConstants;
import frc.robot.intake.IntakeConstants.FrontRollerConstants;
import frc.robot.intake.IntakeConstants.RollerConstants;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Rollers. */
  private final VelocityControllerIO frontRoller, backRoller;

  /** Roller values. */
  private final VelocityControllerIOValues frontRollerValues, backRollerValues;

  private double rollerGoal;

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    frontRoller = IntakeFactory.createFrontRoller();
    frontRollerValues = new VelocityControllerIOValues();
    frontRoller.configure(FrontRollerConstants.CONTROLLER_CONSTANTS);

    backRoller = IntakeFactory.createBackRoller();
    backRollerValues = new VelocityControllerIOValues();
    backRoller.configure(BackRollerConstants.CONTROLLER_CONSTANTS);
  }

  /**
   * Gets the instance of the intake subsystem.
   *
   * @return the instance of the intake subsystem.
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

    frontRoller.setSetpoint(rollerGoal);
    backRoller.setSetpoint(rollerGoal);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Front Roller", frontRollerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Back Roller", backRollerValues);
  }

  public double getRollerVelocity() {
    return (frontRollerValues.velocityRotationsPerSecond + backRollerValues.velocityRotationsPerSecond) / 2;
  }

  public void setGoal(double rollerVelocityRotationsPerSecond) {
    this.rollerGoal = rollerVelocityRotationsPerSecond;
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        frontRollerValues.velocityRotationsPerSecond,
        rollerGoal,
        RollerConstants.SPEED_TOLERANCE)
    && MathUtil.isNear(
        backRollerValues.velocityRotationsPerSecond,
        rollerGoal,
        RollerConstants.SPEED_TOLERANCE);
  }
}
