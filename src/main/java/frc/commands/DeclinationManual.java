// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DeclinationSubsystem;

public class DeclinationManual extends Command {
  DeclinationSubsystem declination;
  CommandXboxController controller;
  /** Creates a new DeclinationManualDown. */
  public DeclinationManual(DeclinationSubsystem declination, CommandXboxController controller) {
    this.declination = declination;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(declination);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    declination.declinationDefaultMethod(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    declination.setPitchVoltage(Volts.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
