// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDrive;
// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

public class FollowPathCommand extends Command {

  Timer timer = new Timer();

  PathPlannerTrajectory trajectory;

  Pose2d currentPose;

  PathPlannerTrajectory.State lastState, wantedState;

  static PIDController translationPID;
  static PIDController rotationPID;

  static {
    translationPID = new PIDController(5, 0, 0);
    rotationPID = new PIDController(5, 0, 0);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData(translationPID);
    SmartDashboard.putData(rotationPID);
  }

  /**
   * Follows a PathPlannerTrajectory
   *
   * @param trajectory the PathPlannerTrajectory
   */
  public FollowPathCommand(PathPlannerTrajectory trajectory) {

    this.trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveDrive.getInstance());

    // RobotState.getInstance()
    //     .setOdometry(
    //         new Pose2d(
    //             trajectory.getInitialState().positionMeters,
    // trajectory.getInitialState().heading));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotationPID.reset();
    translationPID.reset();

    timer.reset();
    timer.start();

    lastState = trajectory.getInitialState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = RobotState.getInstance().getPose();

    /** The current time. */
    double currentTime = timer.get();

    /** Get the state of the robot at this current time in the path. */
    State wantedState = trajectory.sample(currentTime);
    Rotation2d wantedHeading = wantedState.heading;

    double xSpeed = wantedState.velocityMps * Math.cos(wantedHeading.getRadians());
    double ySpeed = wantedState.velocityMps * Math.sin(wantedHeading.getRadians());

    double xPID =
        translationPID.calculate(currentPose.getX(), wantedState.getTargetHolonomicPose().getX());
    double yPID =
        translationPID.calculate(currentPose.getY(), wantedState.getTargetHolonomicPose().getY());

    double wantedRotationSpeeds =
        rotationPID.calculate(currentPose.getRotation().getRadians(), wantedHeading.getRadians());

    Logger.recordOutput("xSpeed + xPID", xSpeed + xPID);
    Logger.recordOutput("ySpeed + yPID", ySpeed + yPID);
    Logger.recordOutput("wantedRotationSpeeds", wantedRotationSpeeds);

    /** Add alliance transform! */

    /** Create a ChassisSpeeds object to represent how the robot should be moving at this time. */
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed + xPID,
            ySpeed + yPID,
            wantedRotationSpeeds,
            RobotState.getInstance().getPoseRotation2d());
    // SwerveDrive.getInstance()
    //     .swerveFollower
    //     .calculateRobotRelativeSpeeds(RobotState.getInstance().currentPose, wantedState);

    Logger.recordOutput(
        "wantedAutoPose", new Pose2d(wantedState.positionMeters, wantedState.heading));

    SwerveDrive.getInstance().driveRobotRelative(chassisSpeeds);
    // SwerveDrive.getInstance().setModuleStates(moduleStates);
    // Logger.recordOutput("Autonomous Set moduleStates", moduleStates);
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
    // return false;

    return timer.hasElapsed(trajectory.getTotalTimeSeconds())
        && Math.abs(
                RobotState.getInstance().getPoseAngleDegrees()
                    - trajectory.getEndState().heading.getDegrees())
            < 1;
    // return timer.hasElapsed(20);
  }
}
