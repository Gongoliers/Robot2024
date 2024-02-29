package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.Arm;
import frc.robot.swerve.Swerve;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  private Swerve swerve;

  @Override
  public void robotInit() {
    robotContainer = RobotContainer.getInstance();
    swerve = Swerve.getInstance();

    new Trigger(this::isEnabled).whileTrue(Arm.getInstance().home());

    new Trigger(this::isDisabled)
        .debounce(RobotConstants.DISABLE_COAST_DELAY)
        .onTrue(Commands.runOnce(() -> swerve.setBrake(false), swerve).ignoringDisable(true));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    swerve.setBrake(true);

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    swerve.setBrake(true);

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
