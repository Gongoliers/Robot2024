package frc.robot.swerve;

import frc.lib.CAN;
import java.util.Objects;

/** CAN identifiers for a swerve module. */
public record SwerveModuleCAN(CAN azimuth, CAN steer, CAN drive) {

  /**
   * Creates the CAN identifiers for a swerve module.
   *
   * @param azimuth the CAN identifier for the swerve module's azimuth encoder.
   * @param steer the CAN identifier for the swerve module's steer motor.
   * @param drive the CAN identifier for the swerve module's drive motor.
   */
  public SwerveModuleCAN {
    Objects.requireNonNull(azimuth);
    Objects.requireNonNull(steer);
    Objects.requireNonNull(drive);
  }

  /**
   * Creates the CAN identifier for a swerve module using numeric identifiers.
   *
   * @param azimuth the numeric identifier assigned to the swerve module's azimuth encoder.
   * @param steer the numeric identifier assigned to the swerve module's steer motor.
   * @param drive the numeric identifier assigned to the swerve module's drive motor.
   * @param bus the CAN bus that the swerve module's devices are located on.
   */
  public SwerveModuleCAN(int azimuth, int steer, int drive, String bus) {
    this(new CAN(azimuth, bus), new CAN(steer, bus), new CAN(drive, bus));
  }
}
