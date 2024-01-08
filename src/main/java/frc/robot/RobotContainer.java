// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.shooter.Shooter;

public class RobotContainer {

  private final Shooter shooter = Shooter.getInstance();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    initializeShuffleboards();
    configureBindings();
  }

  private void initializeShuffleboards() {
    Telemetry.initializeShuffleboard(shooter);
  }

  private void configureBindings() {
    // TODO
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
