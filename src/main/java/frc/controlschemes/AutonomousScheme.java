// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.controlschemes;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.ControlScheme;
import frc.maps.Constants;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class AutonomousScheme implements ControlScheme {

  static Command autoCommand;

  /**
   * Main method called in RobotContainer INSTEAD of configureChoreoBuilder() to allow the robot to
   * follow .traj files in src\main\deploy.
   */
  public static void configurePathPlannerBuilder(
      SwerveDrive swerveDrive,
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController controller,
      AimSimulator aimer) {

    System.out.println("Configuring Pathplanner...");

    AutoBuilder.configureHolonomic(
        swerveDrive::getPose, // Robot pose supplier
        swerveDrive
            ::setOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        swerveDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        swerveDrive::driveRobotRelative,
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            // new PIDConstants(
            //     swerveDrive.xPID.getP(),
            //     swerveDrive.xPID.getI(),
            //     swerveDrive.xPID.getD()), // Translation PID constants
            // new PIDConstants(
            //     swerveDrive.turnPID.getP(),
            //     swerveDrive.turnPID.getI(),
            //     swerveDrive.turnPID.getD()), // Rotation PID constants
            new PIDConstants(1, 0, 0), // Translation PID constants
            new PIDConstants(1, 0, 0),
            Constants.SwerveConstants
                .MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL, // Max module speed, in m/s
            0.44, // Drive base radius in meters. Distance from robot center to furthest module.
            // (0.444522677)
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        swerveDrive // Reference to this subsystem to set requirements
        );

    /** Command List for autos in SmartDashBoard */
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoCommand = autoChooser.getSelected();
    System.out.println("Configured! Command is " + autoCommand.getName());

    registerCommands(
        swerveDrive,
        indexer,
        declination,
        pneumatics,
        rightAscension,
        controller,
        controller,
        aimer);
  }

  static String[] allAutos =
      Arrays.stream(Filesystem.getDeployDirectory().list())
          .map(s -> s.endsWith(".chor") ? s.substring(0, s.length() - 5) : s)
          .toArray(String[]::new);

  /**
   * Main method called in RobotContainer INSTEAD of configurePathPlannerBuilder() to allow the
   * robot to follow .chor files in src\main\deploy. We do not use this method because Choreo
   * currently does not support adding commands to an autonomous routine.
   */
  public static void configureChoreoBuilder(
      SwerveDrive swerveDrive,
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController controller,
      AimSimulator aimer) {

    LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto Choices");
    String autoSelected = chooser.get();

    for (String autonomous : allAutos) {
      // Do your stuff here
      System.out.println(autonomous);
      chooser.addOption(autonomous, autonomous);
    }

    /*
     * Configure Choreo
     */
    ChoreoTrajectory traj = Choreo.getTrajectory(autoSelected);

    Choreo.choreoSwerveCommand(
        traj,
        () -> swerveDrive.getPose(),
        swerveDrive.xPID,
        swerveDrive.yPID,
        swerveDrive.turnPID,
        (ChassisSpeeds speeds) -> {
          SwerveModuleState[] moduleStates;
          // Convert chassis speeds to individual module states
          moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
          swerveDrive.setModuleStates(moduleStates);
        },
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, //
        swerveDrive //
        );

    registerCommands(
        swerveDrive,
        indexer,
        declination,
        pneumatics,
        rightAscension,
        controller,
        controller,
        aimer);
  }

  public static Command getAutoCommand() {
    System.out.println("Recieved Auto Command: " + autoCommand.getName());
    return autoCommand;
  }

  public static void registerCommands(
      SwerveDrive swerveDrive,
      IndexingSubsystem reloading,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController controller,
      CommandXboxController controller2,
      AimSimulator aimer) {}

  /* Helper Methods */

  /**
   * Default end state of 0 mps and 0 degrees
   *
   * @param poses an array of poses to have on the path
   * @return A PathPlannerPath following given poses
   */
  public PathPlannerPath onTheFlyPath(Pose2d[] poses) {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

    // Create the path using the bezier points created above
    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this
            // path. If using a
            // differential drivetrain, the
            // angular constraints have no
            // effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a
            // holonomic rotation here. If using
            // a differential drivetrain, the
            // rotation will have no effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    return path;
  }

  /**
   * @param poses an array of poses to have on the path
   * @param desiredEndState the desired end state of the path in mps and degrees
   * @return an on the fly PathPlannerPath
   */
  public PathPlannerPath onTheFlyPathFromPoses(Pose2d[] poses, GoalEndState desiredEndState) {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

    // Create the path using the bezier points created above
    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            Constants.SwerveConstants.AUTO_PATH_CONSTRAINTS, // The constraints for this
            // path. If using a
            // differential drivetrain, the
            // angular constraints have no
            // effect.
            desiredEndState // Goal end state. You can set a
            // holonomic rotation here. If using
            // a differential drivetrain, the
            // rotation will have no effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    return path;
  }

  /**
   * Follows a single PathPlannerPath
   *
   * @param path the PathPlannerPath to be followed
   * @return The Command to follow the path
   */
  public Command followPathPlannerPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  /**
   * Generates a path to go to target pose
   *
   * @param targetPose the pose that you want to go to. Position and Rotation
   * @return An auto built command to get from current pose to target pose
   */
  /** TODO add rumble */
  public Command generatePathFindToPose(Pose2d targetPose) {
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            Constants.SwerveConstants.AUTO_PATH_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
            );
    return pathfindingCommand;
  }

  public Command pathFindToPathThenFollow(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.pathfindThenFollowPath(
        path, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 0);
  }
}
