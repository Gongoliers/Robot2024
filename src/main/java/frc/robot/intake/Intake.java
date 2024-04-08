package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOConstants;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Roller motors. */
  private final VelocityControllerIO frontRoller, backRoller;

  /** Roller motor values. */
  private final VelocityControllerIOValues frontRollerValues, backRollerValues;

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    frontRoller = IntakeFactory.createFrontRoller();
    frontRollerValues = new VelocityControllerIOValues();
    frontRoller.configure(new VelocityControllerIOConstants());

    backRoller = IntakeFactory.createBackRoller();
    backRollerValues = new VelocityControllerIOValues();
    backRoller.configure(new VelocityControllerIOConstants());
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
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    initializeRollerShuffleboard(tab, "Front Roller", frontRollerValues);
    initializeRollerShuffleboard(tab, "Back Roller", backRollerValues);
  }

  public void initializeRollerShuffleboard(ShuffleboardTab tab, String name, VelocityControllerIOValues values) {
    ShuffleboardLayout roller = Telemetry.addColumn(tab, "Front Roller");

    roller.addDouble("Velocity (rps)", () -> values.velocityRotationsPerSecond);
    roller.addDouble("Voltage (V)", () -> values.motorVolts);
    roller.addDouble("Current (A)", () -> values.motorAmps);
  }

  public double getRollerVelocity() {
    return (frontRollerValues.velocityRotationsPerSecond + backRollerValues.velocityRotationsPerSecond) / 2;
  }

  public void setSetpoint(double rollerVelocityRotationsPerSecond) {
    frontRoller.setSetpoint(rollerVelocityRotationsPerSecond);
    backRoller.setSetpoint(rollerVelocityRotationsPerSecond);
  }
}
