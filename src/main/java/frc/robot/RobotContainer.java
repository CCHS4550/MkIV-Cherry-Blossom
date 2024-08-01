// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.controlschemes.*;
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

  // static SendableChooser<Command> autoChooser;

  /*
   * Initialize controllers.
   */
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
        aimer = new AimSimulator();
        swerveDrive = new SwerveDrive();
        rightAscension = new RightAscensionSubsystem(aimer);
        declination = new DeclinationSubsystem(aimer);
        pneumatics = new PneumaticsSystem();
        indexer = new IndexingSubsystem(pneumatics);

        lights = new Lights();
        break;

      case REPLAY:
        break;
    }

    superstructure = new Superstructure(declination, indexer, lights, pneumatics, rightAscension);

    RobotState.getInstance();

    /*
     * Configure schemes.
     * These configure the controller bindings.
     * Be careful that if you are using the same controller for both schemes, that the controls don't overlap.
     */
    SwerveDriveScheme.configure(swerveDrive, controller1);
    MechanismScheme.configure(
        indexer, declination, pneumatics, rightAscension, controller1, controller2, aimer);

    AutonomousScheme.configurePathPlannerBuilder(
        swerveDrive, indexer, declination, pneumatics, rightAscension, controller1, aimer);

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Path Planner Auto Chooser", autoChooser);

    // CharacterizingScheme.configure(indexer, declination, pneumatics, rightAscension, controller1,
    // aimer);

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("PathPlanner Auto Chooser", autoChooser);
    // autoCommand = autoChooser.getSelected();

    Shuffleboard.getTab("Subsystems").add("SwerveDrive", swerveDrive);
  }

  // public static Command getAutoCommand() {
  //   // System.out.println("Recieved Auto Command: " + autoCommand.getName());
  //   return autoChooser.getSelected();
  // }

  // public static Command getAutoCommand() {
  //   System.out.println("Recieved Auto Command: " + autoCommand.getName());
  //   return autoCommand;
  // }
}
