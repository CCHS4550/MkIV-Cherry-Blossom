// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class FollowPathCommand extends Command {
  Timer timer = new Timer();
  PathPlannerTrajectory trajectory;

  PathPlannerTrajectory.State lastState, currentState, wantedState;

  /** Creates a new FollowPath. */
  public FollowPathCommand(PathPlannerTrajectory trajectory) {
    this.trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveDrive.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    lastState = trajectory.getInitialState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /** The current time. */
    double currentTime = timer.get();

    /** Get the state of the robot at this current time in the path. */
    wantedState = trajectory.sample(currentTime);

    /** Add alliance transform! */
    ChassisSpeeds chassisSpeeds =
        SwerveDrive.getInstance()
            .swerveFollower
            .calculateRobotRelativeSpeeds(RobotState.getInstance().currentPose, wantedState);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    SwerveDrive.getInstance().setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
    SwerveDrive.getInstance().stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    /** This all from 2910 */
    // var currentPose = RobotState.getInstance().currentPose;
    // var desiredPose = trajectory.getEndState().positionMeters;
    // double driveX = RobotState.getInstance().currentPose.getX();
    // double driveY = RobotState.getInstance().currentPose.getY();
    // double driveRotation = RobotState.getInstance().currentPose.getRotation().getRadians();

    // double desiredX = trajectory.getEndState().positionMeters.getX();
    // double desiredY = trajectory.getEndState().positionMeters.getY();
    // double desiredRotation =
    //         trajectory.getEndState().positionMeters.getAngle().getRadians();

    // double xError = Math.abs(desiredX - driveX);
    // double yError = Math.abs(desiredY - driveY);
    // double rotationError = Math.abs(desiredRotation - driveRotation);

    // return (xError < 0.10
    // && yError < 0.10
    // && rotationError < 0.10)
    // ||
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
