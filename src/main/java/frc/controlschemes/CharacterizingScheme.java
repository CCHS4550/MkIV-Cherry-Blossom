// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.maps.ControlMap;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.BarrelRotation;
import frc.robot.subsystems.Declination;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscension;

/** Add your docs here. */
public class CharacterizingScheme {


    public static void configure(
      BarrelRotation barrelRotation,
      Declination declination,
      PneumaticsSystem pneumatics,
      RightAscension rightAscension,
      CommandXboxController controller,
      AimSimulator aimer){


        configureButtons(barrelRotation, declination, pneumatics, rightAscension, controller, controller, aimer);
      }


    public static void configureButtons(
      BarrelRotation barrelRotation,
      Declination declination,
      PneumaticsSystem pneumatics,
      RightAscension rightAscension,
      CommandXboxController controller,
      CommandXboxController controller2,
      AimSimulator aimer){


        controller.a().onTrue(rightAscension.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.b().onTrue(rightAscension.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controller.x().onTrue(rightAscension.sysIdDynamic(SysIdRoutine.Direction.kForward)); 
        controller.y().onTrue(rightAscension.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      }

    

      





}
