// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.climber.Climber;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Swerve;
import frc.robot.vision.Vision;

public class RobotContainer {

  private final Arm arm = Arm.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Swerve swerve = Swerve.getInstance();
  private final Vision vision = Vision.getInstance();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    initializeShuffleboards();
    configureBindings();
  }

  private void initializeShuffleboards() {
    Telemetry.initializeShuffleboards(arm, climber, intake, shooter, swerve, vision);
  }

  private void configureBindings() {
    operator.leftBumper().whileTrue(shooter.intake());
    operator.leftTrigger().whileTrue(shooter.smartIntake());

    operator.rightBumper().whileTrue(shooter.shoot());
    operator.rightTrigger().whileTrue(shooter.smartShoot());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
