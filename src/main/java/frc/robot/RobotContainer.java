// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.controlschemes.MechanismScheme;
import frc.controlschemes.SwerveDriveScheme;
import frc.maps.Constants;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  CommandXboxController controller1 = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);
  AimSimulator aimer;

  SwerveDrive swerveDrive;
  RightAscensionSubsystem rightAscension;
  DeclinationSubsystem declination;
  PneumaticsSystem pneumatics;
  IndexingSubsystem indexer;

  Superstructure superstructure;

  Lights lights;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        aimer = new AimSimulator();
        swerveDrive = new SwerveDrive();
        rightAscension = new RightAscensionSubsystem(aimer);
        declination = new DeclinationSubsystem(aimer);
        pneumatics = new PneumaticsSystem();
        indexer = new IndexingSubsystem(pneumatics);

        lights = new Lights();

        break;

      case SIM:
        break;
    }

    superstructure = new Superstructure(declination, indexer, lights, pneumatics, rightAscension);

    RobotState.getInstance();

    // Be careful that if you are using the same controller for both schemes, that the controls
    // don't overlap.
    SwerveDriveScheme.configure(swerveDrive, controller1);
    MechanismScheme.configure(
        indexer, declination, pneumatics, rightAscension, controller1, controller2, aimer);

    // CharacterizingScheme.configure(
    //     barrelRotation, declination, pneumatics, rightAscension, controller1, aimer);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@links
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
}
