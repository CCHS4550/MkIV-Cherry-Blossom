// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */
public class CustomAutoChooser {

  public static CustomAutoChooser mInstance;

  public static CustomAutoChooser getInstance() {
    if (mInstance == null) {
      mInstance = new CustomAutoChooser();
    }
    return mInstance;
  }

  private enum Autos {
    AUTO1,
    FOLLOW_ONE_METER,
    AUTO3
  }

  // private final PathTrajectories trajectories;

  private static final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  private CustomAutoChooser() {
    // this.trajectories = trajectories;

    autoChooser.setDefaultOption("Auto 1 Name", Autos.AUTO1);
    autoChooser.addOption("Auto 2 Name", Autos.FOLLOW_ONE_METER);
    autoChooser.addOption("Auto 3 Name", Autos.AUTO3);
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

  public Command auto1Command() {
    return follow("test1").andThen(new InstantCommand());
  }

  public Command followOneMeter() {
    return follow("TYLERPATH");
  }

  public Command getSelectedCustomCommand() {

    switch (autoChooser.getSelected()) {
      case AUTO1:
        return auto1Command();
      case FOLLOW_ONE_METER:
        return followOneMeter();
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
  public Command follow(String pathname) {

    return new FollowPathCommand(
        PathPlannerPath.fromChoreoTrajectory(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d()));
  }
}
