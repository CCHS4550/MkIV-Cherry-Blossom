// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.RightAscensionSubsystem;

public class RightAscensionManual extends Command {
  RightAscensionSubsystem rightAscension;

  /** Creates a new RightAscensionManual. */
  public RightAscensionManual(
      RightAscensionSubsystem rightAscension, CommandXboxController controller) {

    this.rightAscension = rightAscension;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rightAscension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
