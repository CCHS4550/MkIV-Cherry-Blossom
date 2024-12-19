// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */
public class CustomAutoChooser {

  //   public static CustomAutoChooser mInstance;

  //   public static CustomAutoChooser getInstance() {
  //     if (mInstance == null) {
  //       mInstance = new CustomAutoChooser();
  //     }
  //     return mInstance;
  //   }

  /** All the Auto Names. */
  private enum Autos {
    FOLLOW_ONE_METER,
    FOLLOW_TWO_METER,
    FOLLOW_ONE_METER_PP,
    WORKSHOP_TEST,
    EMPTY
  }

  // private final PathTrajectories trajectories;

  /**
   * This is what puts the options on Smart Dashboard, but instead of doing it by itself, we have to
   * populate it manually.
   */
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  public CustomAutoChooser() {
    // this.trajectories = trajectories;

    autoChooser.setDefaultOption("Auto 1 Name", Autos.EMPTY);
    autoChooser.addOption("Auto 2 Name", Autos.FOLLOW_ONE_METER);
    autoChooser.addOption("Auto 3 Name", Autos.FOLLOW_TWO_METER);
    autoChooser.addOption("Auto 4 Name", Autos.FOLLOW_ONE_METER_PP);
    autoChooser.addOption("Auto 5 Name", Autos.WORKSHOP_TEST);
    SmartDashboard.putData("Custom AutoChooser", autoChooser);
    // autoChooser.addOption("Auto 3 Name", Autos.AUTO3);
  }

  /**
   * Here is an example of how it should be formatted, give or take. This blob is copied exactly
   * from 2910
   */
  //     private Command followAndDoChargingStationAndHome(RobotContainer container,
  // PathPlannerTrajectory trajectory) {
  //     SequentialCommandGroup homeArmCommand = new SequentialCommandGroup();
  //     homeArmCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(),
  // ArmPoseConstants.STOW));
  //     homeArmCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));

  //     SequentialCommandGroup chargingStationCommand = new SequentialCommandGroup();
  //     chargingStationCommand.addCommands(follow(container, trajectory));
  //     chargingStationCommand.addCommands(new
  // AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));

  //     return chargingStationCommand.alongWith(homeArmCommand);
  // }

  //   public Command auto1Command() {
  //     return follow("test1").andThen(new InstantCommand());
  //   }

  public Command followOneMeter() {
    return followChoreo("TYLERPATH");
  }

  public Command followTwoMeter() {
    return followChoreo("SUHITPATH");
  }

  public Command followOneMeterPP() {
    return followPathPlanner("1Meter");
  }

  public Command workshopTest() {
    return followPathPlanner("Workshop Test");
  }

  public Command getSelectedCustomCommand() {

    switch (autoChooser.getSelected()) {
        //   case AUTO1:
        //     return auto1Command();
      case FOLLOW_ONE_METER:
        return followOneMeter();
      case FOLLOW_TWO_METER:
        return followTwoMeter();
      case EMPTY:
        return new InstantCommand();
      case FOLLOW_ONE_METER_PP:
        return followOneMeterPP();
      case WORKSHOP_TEST:
        return workshopTest();
      default:
        return new InstantCommand();
    }
  }

  /**
   * Creates a simpler follow helper method that simply requires the .traj file name from the
   * deploy/choreo directory.
   *
   * @param pathname - The path name, no extension.
   * @return - A follow Command!
   */
  public Command followChoreo(String pathname) {

    return new FollowPathCommand(
        PathPlannerPath.fromChoreoTrajectory(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d()));
  }

  public Command followPathPlanner(String pathname) {
    return new FollowPathCommand(
        PathPlannerPath.fromPathFile(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d()));
  }
}
