// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands.defaultcommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Declination;

public class DeclinationDefault extends Command {
  CommandXboxController controller;
  Declination declination;
  /** Creates a new DeclinationDefault. */
  public DeclinationDefault(Declination declination, CommandXboxController controller) {
    this.controller = controller;
    this.declination = declination;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(declination);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    declination.declinationDefaultMethod(controller);

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
