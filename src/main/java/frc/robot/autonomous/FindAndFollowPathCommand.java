// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class FindAndFollowPathCommand extends Command {

  private LocalADStar pathFinder = new LocalADStar();

  PathConstraints pathConstraints = new PathConstraints(2, 1, .5, .5);

  Timer timer = new Timer();
  
  Pose2d targetPose = new Pose2d();

  PathPlannerPath path;
  PathPlannerTrajectory trajectory;
  
  /** Creates a new FindAndFollowPathCommand. */
  public FindAndFollowPathCommand(Pose2d targetPose) {
    this.targetPose = targetPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    pathFinder.setStartPosition(RobotState.getInstance().getPose().getTranslation());
    pathFinder.setGoalPosition(targetPose.getTranslation());
    

    path = pathFinder.getCurrentPath(pathConstraints, new GoalEndState(0, targetPose.getRotation()));

    trajectory = path.getTrajectory(
      Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(SwerveDrive.getInstance().desiredModuleStates), 
      RobotState.getInstance().getPoseRotation2d());

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
