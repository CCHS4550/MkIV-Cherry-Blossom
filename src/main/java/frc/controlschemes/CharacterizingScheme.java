// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;

/** Add your docs here. */
public class CharacterizingScheme {

  public static void configure(
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController controller,
      AimSimulator aimer) {

    configureButtons(
        indexer, declination, pneumatics, rightAscension, controller, controller, aimer);
  }

  public static void configureButtons(
      IndexingSubsystem reloading,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController controller,
      CommandXboxController controller2,
      AimSimulator aimer) {

    controller.a().onTrue(rightAscension.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.b().onTrue(rightAscension.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.x().onTrue(rightAscension.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.y().onTrue(rightAscension.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
