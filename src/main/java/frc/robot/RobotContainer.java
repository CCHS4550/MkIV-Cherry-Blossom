// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.controlschemes.CharacterizingScheme;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.Reloading;
import frc.robot.subsystems.Declination;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscension;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  CommandXboxController controller1 = new CommandXboxController(0);
  AimSimulator aimer = new AimSimulator();

  SwerveDrive swerveDrive = new SwerveDrive();
  RightAscension rightAscension = new RightAscension(aimer, controller1);
  Declination declination = new Declination(aimer, controller1);
  PneumaticsSystem pneumatics = new PneumaticsSystem();
  Reloading reload = new Reloading(pneumatics);
  // Lights lights = new Lights();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

<<<<<<< HEAD
    // Be careful that if you are using the same controller for both schemes, that the controls
    // don't overlap.
    // SwerveDriveScheme.configure(swerveDrive, controller1);
    // MechanismScheme.configure(
    //     barrelRotation, declination, pneumatics, rightAscension, controller1, aimer);
=======
    // Be careful that if you are using the same controller for both schemes, that the controls don't overlap.
    SwerveDriveScheme.configure(swerveDrive, controller1);
    MechanismScheme.configure(reload, declination, pneumatics, rightAscension, controller1, aimer);
>>>>>>> 76be97e9bda9e22112632a3041041561e3db6d15

    CharacterizingScheme.configure(
        barrelRotation, declination, pneumatics, rightAscension, controller1, aimer);
    // initialize controller schemes here
    //  SwerveDriveScheme.configure(swerveDrive, 0);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup();
  }

  private void defaultCommands() {}
  ;
}
