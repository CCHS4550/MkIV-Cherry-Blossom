// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.controlschemes.*;
import frc.maps.Constants;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PhotonVision;
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
  PhotonVision vision;

  SwerveDrive swerveDrive;
  RightAscensionSubsystem rightAscension;
  DeclinationSubsystem declination;
  PneumaticsSystem pneumatics;
  IndexingSubsystem indexer;

  Lights lights;

  Superstructure superstructure;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        aimer = AimSimulator.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        rightAscension = RightAscensionSubsystem.getInstance();
        declination = DeclinationSubsystem.getInstance();
        pneumatics = PneumaticsSystem.getInstance();
        indexer = IndexingSubsystem.getInstance();
        vision = PhotonVision.getInstance();

        lights = Lights.getInstance();

        break;

      case SIM:
        aimer = AimSimulator.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        rightAscension = RightAscensionSubsystem.getInstance();
        declination = DeclinationSubsystem.getInstance();
        pneumatics = PneumaticsSystem.getInstance();
        indexer = IndexingSubsystem.getInstance();

        lights = Lights.getInstance();

        break;

      case REPLAY:
        break;
    }

    RobotState.getInstance();
    RobotState.getInstance().dashboardInit();
    RobotState.getInstance().poseInit();

    // superstructure = new Superstructure(declination, indexer, , pneumatics, rightAscension);

    /*
     * Configure schemes.
     * These configure the controller bindings.
     * Be careful that if you are using the same controller for both schemes, that the controls don't overlap.
     */
    SwerveDriveScheme.configure(swerveDrive, controller1);
    MechanismScheme.configure(
        swerveDrive,
        indexer,
        declination,
        pneumatics,
        rightAscension,
        controller1,
        controller2,
        aimer);

    AutoBuilderScheme.configurePathPlannerBuilder(
        swerveDrive, indexer, declination, pneumatics, rightAscension, controller1, aimer);

    // CharacterizingScheme.configure(
    //     swerveDrive, indexer, declination, pneumatics, rightAscension, controller1, aimer);
  }
}
