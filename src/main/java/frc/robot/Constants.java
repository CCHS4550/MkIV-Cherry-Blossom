package frc.maps;
package frc.robot;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.helpers.AllianceFlipUtil;
//took imports from Constants.java, 4550-Crescendo-2024 repo
//remove as needed



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class MotorConstants{
    /* MAKE SURE THE MOTOR NUMBERS ARE CORRECT */
    //ASSUMING THAT IT WOULD GO FR, FL, BR, BL, LIKE IN OTHER SWERVE STUFF
    //ENCODERS ARE 1, CHECK THAT
    public static final int FRONT_RIGHT_DRIVE = 1;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
    public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
    public static final int FRONT_RIGHT_TURN = 2;
    public static final boolean FRONT_RIGHT_TURN_REVERSE = false;
    public static final double FRONT_RIGHT_TURN_ENCODER = 1;

     public static final int FRONT_LEFT_DRIVE = 3;
    public static final boolean FRONT_LEFT_DRIVE_REVERSE = false;
    public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
    public static final int FRONT_LEFT_TURN = 4;
    public static final boolean FRONT_LEFT_TURN_REVERSE = false;
    public static final double FRONT_LEFT_TURN_ENCODER = 1;

     public static final int BACK_RIGHT_DRIVE = 5;
    public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
    public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
    public static final int BACK_RIGHT_TURN = 6;
    public static final boolean BACK_RIGHT_TURN_REVERSE = false;
    public static final double BACK_RIGHT_TURN_ENCODER = 1;

     public static final int BACK_LEFT_DRIVE = 7;
    public static final boolean BACK_LEFT_DRIVE_REVERSE = false;
    public static final double BACK_LEFT_DRIVE_ENCODER = 1;
    public static final int BACK_LEFT_TURN = 8;
    public static final boolean BACK_LEFT_TURN_REVERSE = false;
    public static final double BACK_LEFT_TURN_ENCODER = 1;

    //CHECK MOTOR IDs, probably wrong
    public static final int YAW_1 = 9;
    public static final boolean YAW_1_REVERSE = false;
    public static final int YAW_2 = 10;
    public static final boolean YAW_2_REVERSE = false;

    public static final int ROLL = 11; 
    public static final boolean ROLL_REVERSE = false;

    public static final int PITCH = 12;
    public static final boolean PITCH_REVERSE = false;

  }

  public static class SwerveConstants{
    //ports for aabsolute encoders
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 0;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER =1;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 2;
    public static final int BACK_LEFT_ANSOLUTE_ENCODER = 3;
    //absolute encoder offsets
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 0;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 0;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 0;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 0;

    //Robot Constants (change/calculate with SysId)
    //max speed in free sprint: used in getting velocities of swerve modules
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL = 0; //calculate this, on crescendo repo it's 4.72


    //velocity limtis
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    //Rate/Acceleraation Limiters
    public static final double DRIVE_RATE_LIMIT = MAX_DRIVE_SPEED_METERS_PER_SECOND * 1.5;
    public static final double TURN_RATE_LIMIT = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    public static final TrapezoidProfile.Constraints thetaControlConstraints = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TURN_RATE_LIMIT);
    public static final TrapezoidProfile.Constraints driveControlConstraints  =  new TrapezoidalProfile.Constraints(MAX_DRIVE_SPEED_METERS_PER_SECOND, DRIVE_RATE_LIMIT);
`   //Front to Back
    public static final double WHEEL_BASE = Units.inchesToMeters(0);
    //Right to Left
    public static final double TRACK_WIDTH = Units.inchesToMeters(0);

    SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics (
      new Translation2d (WHEEL_BASE/2, -TRACK_WIDTH/2),
      new Translation2d (WHEEL_BASE/2, TRACK_WIDTH/2),
      new Translation2d (-WHEEL_BASE/2, -TRACK_WIDTH/2),
      new Translation2d(-WHEEL_BASE/2, TRACK_WIDTH/2)
    );
  }

  

  public static class SwerveConversionConstants{

    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4*Math.PI);

    //turn motor constants
    //150/7 rotations of turn motor = 1 spin of the wheel
    //how much of a rotation the wheel turns for one rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 7.0/150.0;
    //150/7 rotations converted into radians
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS = Units.rotationsToRadians(TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS);
    //radians per second
    public static final double TURN_MOTOR_RADIANS_PER_SECOND = TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS/60.0;

    //drive motor constants
    //6.75 rotations of drive motor to 1 wheel spin
    public static final double DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1.0/6.75;
    //distance traveled per rotation
    public static final double HORIZONTAL_DISTANCE_TRAVELED_PER_MOTOR_REVOLUTION = WHEEL_CIRCUMFERENCE*DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS;
    public static final double HORIZONTAL_DISTANCE_TRAVELED_PER_MOTOR_REVOLUTION_PER_SECOND = HORIZONTAL_DISTANCE_TRAVELED_PER_MOTOR_REVOLUTION/60.0;

    

  }


  public static class FeedForwardConstants{

  }
}
