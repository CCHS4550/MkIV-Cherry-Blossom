// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */


/** Largely inspired of off  */
public class PathWrapper {

    // This is what is input to the AutoWrapper
    public record AutoFile(String fileName, boolean isChoreoTraj) {}

    // This is what is created.
    // private record PathData(PathPlannerTrajectory trajectory, Command autoCommand) {}

    ArrayList<Command> followCommands = new ArrayList<>();

    PathPlannerTrajectory initialTraj;
    Pose2d initialPose;


    /** Wraps all the paths associated with an Autonomous routine inside a container object.
     *  @param pathFiles - All the file names of the .traj files, in order by use.
     *
     */
    public PathWrapper(AutoFile... files) {

        initialTraj = files[0].isChoreoTraj ? getChoreoTrajectory(files[0].fileName) : getPathPlannerTrajectory(files[0].fileName);
        initialPose = new Pose2d(initialTraj.getInitialState().positionMeters, initialTraj.getInitialState().heading);
            
        
        
        for (AutoFile file : files) {
            
            if (file.isChoreoTraj) {
                followCommands.add(followChoreo(file.fileName));
            } else {
                followCommands.add(followPathPlanner(file.fileName));
            }
        }
  
    

        
    }

    public Command setInitialPose() {
    return new InstantCommand(() -> 
    RobotState.getInstance()
        .setOdometry(initialPose));
    }

    public Command getStartingCommand() {
        return followCommands.get(0);
    }
    public Command getEndingCommand() {
        return followCommands.get((followCommands.size() - 1));
    }

    public Command getFollowCommand(int index) {
        return followCommands.get(index);
    }




    /**
     * 
     *
     * Creates a simpler follow helper method that simply requires the .traj file name from the
     * deploy/choreo directory.
     *
     * @param pathname - The path name, no extension.
     * @return - A follow Command!
     */
    private static Command followChoreo(String pathname) {

    return new FollowPathCommand(
        PathPlannerPath.fromChoreoTrajectory(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d()));
    }

    private static Command followPathPlanner(String pathname) {
    return new FollowPathCommand(
        PathPlannerPath.fromPathFile(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d()));
    }

    private static PathPlannerTrajectory getChoreoTrajectory(String pathname) {
    return PathPlannerPath.fromChoreoTrajectory(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d());
     }
    private static PathPlannerTrajectory getPathPlannerTrajectory(String pathname) {
    return PathPlannerPath.fromPathFile(pathname)
            .getTrajectory(
                SwerveDrive.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getRotation2d());
     }
}
