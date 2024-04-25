package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.ControllerConstants;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOConstants;

/** Constants for the intake subsystem. */
public class IntakeConstants {
  /** Constants for the front roller. */
  public static class FrontRollerConstants {
    /** Front roller's CAN. */
    public static final CAN CAN = new CAN(50);

    /** Front roller's PIDF constants. */
    public static final ControllerConstants PIDF_CONTROLLER_CONSTANTS =
        new ControllerConstants()
            .withFeedforward(
                new FeedforwardControllerConfig()
                    .withStaticFeedforward(0.13) // volts
                    .withVelocityFeedforward(0.1683) // volts per rotation per second
                )
            .withFeedback(
                new FeedbackControllerConfig()
                    .withProportionalGain(0.1) // volts per rotation per second
                );

    /** Front roller's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = false;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 24.0 / 16.0;
    }

    public static final double INTAKE_SPEED = 34;

    public static final double EJECT_SPEED = -34;

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 67;

    public static final double NOTE_AMPS = 40;
  }

  /** Constants for the back roller. */
  public static class BackRollerConstants {
    /** Back roller's CAN. */
    public static final CAN CAN = new CAN(40);

    public static final ControllerConstants PIDF_CONTROLLER_CONSTANTS =
        new ControllerConstants()
            .withFeedforward(
                new FeedforwardControllerConfig()
                    .withStaticFeedforward(0.13) // volts
                    .withVelocityFeedforward(0.1759) // volts per rotation per second
                )
            .withFeedback(
                new FeedbackControllerConfig()
                    .withProportionalGain(0.1) // volts per rotation per second
                );

    /** Back roller's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = false;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 24.0 / 16.0;
    }

    public static final double INTAKE_SPEED = 34;

    public static final double EJECT_SPEED = -34;

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 67;

    public static final double NOTE_AMPS = 40;
  }
}
