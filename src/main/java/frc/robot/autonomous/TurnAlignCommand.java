// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class TurnAlignCommand extends Command {

  PhotonTrackedTarget target;
  
  /** Creates a new TurnAlign. */
  public TurnAlignCommand() {

    target = PhotonVision.getInstance().frontCamera.getLatestResult().getBestTarget();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveDrive.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ChassisSpeeds chassisSpeeds =
        SwerveDrive.getInstance()
            .swerveFollower
            .calculateRobotRelativeSpeeds(new Pose2d(0, 0, RobotState.getInstance().getAngleBetweenCurrentAndTargetPose(new Pose2d(0, 0,Rotation2d.fromDegrees(target.getYaw())))), new State());
                // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    SwerveDrive.getInstance().setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotState.getInstance().getAngleBetweenCurrentAndTargetPose(new Pose2d()).getDegrees()) < 2) || target == null;
  }
}
