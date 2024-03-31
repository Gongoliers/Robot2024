package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.intake.RollerMotorIO.RollerMotorIOValues;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Roller motor. */
  private final RollerMotorIO rollerMotor;

  /** Roller motor values. */
  private final RollerMotorIOValues rollerMotorValues = new RollerMotorIOValues();

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    rollerMotor = IntakeFactory.createRollerMotor();

    rollerMotor.configure();
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
    rollerMotor.update(rollerMotorValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout roller = Telemetry.addColumn(tab, "Roller");

    roller.addDouble("Current (A)", () -> rollerMotorValues.currentAmps);
    roller.addDouble("Velocity (rps)", () -> rollerMotorValues.velocityRotationsPerSecond);
  }

  public double getRollerVelocity() {
    rollerMotor.update(rollerMotorValues);

    return rollerMotorValues.velocityRotationsPerSecond;
  }

  public void setSetpoint(double rollerVelocityRotationsPerSecond) {
    rollerMotor.setSetpoint(rollerVelocityRotationsPerSecond);
  }
}
