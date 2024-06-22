// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.controlschemes;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.ControlScheme;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class SwerveDriveScheme implements ControlScheme{
private static CommandXboxController controller;

public static void configure(SwerveDrive swervedrive, int port){
controller = new CommandXboxController(port);

configureButtons(swervedrive, port);
}

private static void configureButtons(SwerveDrive swervedrive, int port){}



}
