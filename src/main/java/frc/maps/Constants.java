package frc.maps;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import frc.helpers.AllianceFlipUtil;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

  public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(4 * Math.PI);

  public static Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static void getCurrentMode() {
    if (RobotBase.isReal()) {
      currentMode = Mode.REAL;
    } else if (RobotBase.isSimulation()) {
      currentMode = Mode.SIM;
    }

    // return currentMode;
  }

  public static class ConversionConstants {

    // 150/7 rotations of the turn motor to one rotation of the wheel
    // how much of a rotation the wheel turns for one rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 7.0 / 150.0;

    // How many radians the wheel pivots for one full rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS =
        Units.rotationsToRadians(TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS);
    public static final double TURN_MOTOR_RADIANS_PER_SECOND =
        TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS / 60.0;

    // 6.75 rotations of the drive motor to one spin of the wheel
    public static final double DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1.0 / 6.75;
    // horizontal distance travelled by one motor rotation
    public static final double HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION =
        WHEEL_CIRCUMFRENCE * DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS;

    public static final double DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR =
        HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION / 60.0;
  }

  public static class MotorConstants {

    // Swerve Drive Base Motor & CAN ID Constants
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSE = true;
    public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
    public static final int FRONT_RIGHT_TURN = 4;
    public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
    public static final double FRONT_RIGHT_TURN_ENCODER = 1;

    public static final int FRONT_LEFT_DRIVE = 7;
    public static final boolean FRONT_LEFT_DRIVE_REVERSE = true;
    public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
    public static final int FRONT_LEFT_TURN = 6;
    public static final boolean FRONT_LEFT_TURN_REVERSE = true;
    public static final double FRONT_LEFT_TURN_ENCODER = 1;

    public static final int BACK_RIGHT_DRIVE = 2;
    public static final boolean BACK_RIGHT_DRIVE_REVERSE = true;
    public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
    public static final int BACK_RIGHT_TURN = 1;
    public static final boolean BACK_RIGHT_TURN_REVERSE = true;
    public static final double BACK_RIGHT_TURN_ENCODER = 1;

    public static final int BACK_LEFT_DRIVE = 9;
    public static final boolean BACK_LEFT_DRIVE_REVERSE = true;
    public static final double BACK_LEFT_DRIVE_ENCODER = 1;
    public static final int BACK_LEFT_TURN = 8;
    public static final boolean BACK_LEFT_TURN_REVERSE = true;
    public static final double BACK_LEFT_TURN_ENCODER = 1;

    public static final int RIGHT_ASCENSION = 10;
    public static final int[] DECLINATION = {11, 12};
    public static final int BARREL_ROTATION = 13;
  }

  public static class SwerveConstants {

    // Absolute Encoder Ports
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 3;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 0;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER = 2;

    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 2.348;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 5.6393;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 1.7729;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 5.965215;

    // Robot Constants (change with SysId)
    // max speed in free sprint: used in getting velocities of swerve modules
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL = 3.71;

    // Velocity Limits
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    // Rate Limiters (acceleration)
    public static final double DRIVE_RATE_LIMIT = MAX_DRIVE_SPEED_METERS_PER_SECOND * 1.5;
    public static final double TURN_RATE_LIMIT = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    public static final PathConstraints AUTO_PATH_CONSTRAINTS =
        new PathConstraints(
            MAX_DRIVE_SPEED_METERS_PER_SECOND - 2,
            DRIVE_RATE_LIMIT - 0.3,
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            TURN_RATE_LIMIT);
    // public static final PathConstraints AUTO_PATH_CONSTRAINTS = new
    // PathConstraints(4, 3);
    public static final TrapezoidProfile.Constraints thetaControlConstraints =
        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TURN_RATE_LIMIT);

    // Robot Dimensions (relative to wheel locations)
    // Since this robot is a square, no need for 2 values. In a non-square chassis,
    // 2 values needed.

    // Front to back
    public static final double WHEEL_BASE =
        Units.inchesToMeters(24.750000); // from drive shaft to drive shaft. Previous was
    // Right to Left                                                            // 27
    public static final double TRACK_WITDTH = Units.inchesToMeters(24.750000);

    public static final double RADIUS = Math.sqrt(2) * (WHEEL_BASE / 2);
    /** FR FL BR BL. Same as order of swerve module states */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WITDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WITDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WITDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WITDTH / 2));
  }

  public static class PneumaticsConstants {
    public static final int COMPRESSOR_FAN = 14;
    public static final int[] PRESSURE_SEAL = {9, 10};
    public static final int SHOOTING_VALVE = 8;
  }
  /* To Do */
  public static final int LED_PORT = 0;
  public static final int LED_LENGTH = 50;

  public static class SensorMiscConstants {
    public static final int BARREL_SENSOR = 0;
    public static final int YAW_SENSOR = 1;
    public static final int LEDS = 2;
    public static final int PITCH_LIMIT_SWITCH = 2;
  }

  public class FeedForwardConstants {

    // TODO Do sysid to get values
    public static final double DRIVE_KS = 0.16681;
    public static final double DRIVE_KV = 2.609;
    public static final double DRIVE_KA = 0.51582;

    public static final double TURNKS = 0;
    public static final double TURNKV = 0;

    // /* TODO SysId these values */

    // KS should be >= 0, so we'll override it to 0.
    public static final double INDEX_KS = 0;
    //  -10.806;
    public static final double INDEX_KV = 2.0129;
    public static final double INDEX_KA = 4.5878;

    // conversion by 0.25 (Not Ideal)
    public static final double DECLINATION_KS = 0.16328;
    public static final double DECLINATION_KV = 0.0029191;
    public static final double DECLINATION_KA = 0.0016397;
    public static final double DECLINATION_KG = 0.15212;

    // conversion by 0.0125 (Ideal)
    // Doesn't work oddly enough
    // public static final double DECLINATION_KS = 0.16117;
    // public static final double DECLINATION_KV = 0.11154;
    // public static final double DECLINATION_KA = 0.015231;
    // public static final double DECLINATION_KG = 0.79476;

    public static final double RIGHT_ASCENSION_KS = 0;
    // -0.081946;
    public static final double RIGHT_ASCENSION_KV = 1.2324;
    public static final double RIGHT_ASCENSION_KA = 0.63117;

    // Free Running Motor
    public static final double RIGHT_ASCENSION_KS_TEST = 0.076895;
    public static final double RIGHT_ASCENSION_KV_TEST = 0.0019762;
    public static final double RIGHT_ASCENSION_KA_TEST = 0.00024605;
  }

  public class FieldPositionConstants {
    /** Blue Top to Bottom then Red Top to Bottom */
    public static Pose2d[] blueNotePoses =
        new Pose2d[] {
          new Pose2d(2.89, 7.0, null),
          new Pose2d(2.89, 5.53, null),
          new Pose2d(2.89, 4.10, new Rotation2d(0))
        };

    // public static Pose2d[] redNotePoses = new Pose2d[]{new Pose2d(2.89, 7.0,
    // null), new Pose2d(2.89, 5.53, null),
    // new Pose2d(2.89, 4.10, new Rotation2d(0))}
  }

  public class MechanismPositions {
    public static double ELEVATOR_INTAKE = 0;
    public static double WRIST_INTAKE = 0;

    public static double ELEVATOR_SHOOT = 0;
    public static double WRIST_SHOOT = 15.619040489196777;
    // public static double WRIST_SHOOT = 8.714315414428711;

    public static double ELEVATOR_AMP = 77;
    public static double WRIST_AMP = 50.5;

    public static double ELEVATOR_HUMAN_PLAYER = 0;
    public static double WRIST_HUMAN_PLAYER = 0;

    public static double ELEVATOR_TOP = 75;

    public static double WRIST_TRAVEL = 20;
  }

  public class XboxConstants {
    // Joystick Axises
    public static final int L_JOYSTICK_HORIZONTAL = 0;
    public static final int L_JOYSTICK_VERTICAL = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int R_JOYSTICK_HORIZONTAL = 4;
    public static final int R_JOYSTICK_VERTICAL = 5;

    // Controller Buttons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;
    public static final int SELECT_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // These buttons are when you push down the left and right circle pad
    public static final int L_JOYSTICK_BUTTON = 9;
    public static final int R_JOYSTICK_BUTTON = 10;

    // D Pad Buttons
    public static final int DPAD_UP = 0;
    public static final int DPAD_UP_RIGHT = 45;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN_RIGHT = 135;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_DOWN_LEFT = 225;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_UP_LEFT = 315;

    // Controller Zeroes
    public static final double ZERO = 0.15;
  }

  // public static class Vision {
  //   public static final String CAMERA_NAME = "FrontCamera";
  //   // Cam mounted facing forward, half a meter forward of center, half a meter up
  //   // from center.

  //   // The layout of the AprilTags on the field

  //   // The standard deviations of our vision estimated poses, which affect
  //   // correction rate
  //   // (Fake values. Experiment and determine estimation noise on an actual robot.)
  //   public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  //   public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  // }

  public static class cameraOne {
    public static final String CAMERA_ONE_NAME = "FrontCamera";
    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(Inches.of(9.3418), Inches.of(0), Inches.of(14.157)),
            new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));
  }

  /**
   * Gotten from here
   * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
   */
  public static class AprilTags {
    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static int BLUE_SOURCE_LEFT = 1;
    public static int BLUE_SOURCE_RIGHT = 2;
    public static int RED_SPEAKER_BOTTOM = 3;
    public static int RED_SPEAKER_TOP = 4;
    public static int RED_AMP = 5;
    public static int BLUE_AMP = 6;
    public static int BLUE_SPEAKER_TOP = 7;
    public static int BLUE_SPEAKER_BUTTON = 8;
    public static int RED_SOURCE_LEFT = 9;
    public static int RED_SOURCE_RIGHT = 10;
    public static int RED_STAGE_BOTTOM = 11;
    public static int RED_STAGE_TOP = 12;
    public static int RED_STAGE_SIDE = 13;
    public static int BLUE_STAGE_SIDE = 14;
    public static int BLUE_STAGE_TOP = 15;
    public static int BLUE_STAGE_BOTTOM = 16;
  }

  public static Pose2d mirrorPose(Pose2d bluePose) {
    return new Pose2d(
        Constants.AprilTags.aprilTagFieldLayout.getFieldLength() - bluePose.getX(),
        bluePose.getY(),
        Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians()));
  }

  public static class ElevatorConstants {
    public static double ELEVATOR_HEIGHT = 19.345;
    public static double ELEVATOR_ANGLE = 0.611;
  }

  // safely divide
  public double safeDivision(double numerator, double denominator) {
    if (Math.abs(denominator) < 0.00001) {
      return 0.0;
    }
    double dividend = numerator / denominator;
    return dividend;
  }
}
