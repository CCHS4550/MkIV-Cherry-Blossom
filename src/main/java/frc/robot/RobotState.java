// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.maps.Constants;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

/** RobotState is used to retrieve information about the robot's state in other classes. */
public class RobotState {

  public static RobotState instance;
  // Initialize gyro
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  public final Field2d m_field_poseestimator = new Field2d();
  public final Field2d m_field_getPose = new Field2d();

  // SwerveDriveOdometry odometer;
  public SwerveDrivePoseEstimator poseEstimator;
  public PhotonPoseEstimator photonPoseEstimator;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public void poseInit(
      SwerveDrive swerveDrive,
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      AimSimulator aimer) {

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.getAngle() + 180).unaryMinus(),
            swerveDrive.swerveModulePositions,
            new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void updatePose() {

    m_field_poseestimator.setRobotPose(poseEstimator.getEstimatedPosition());
    m_field_getPose.setRobotPose(getPose());

    poseEstimator.update(getRotation2d(), SwerveDrive.swerveModulePositions);
  }

  public void dashboardInit(
      SwerveDrive swerveDrive,
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      AimSimulator aimer) {

    /* Put the Command Scheduler on SmartDashboard */
    SmartDashboard.putData(CommandScheduler.getInstance());

    /* Put all the subsystems on ShuffleBoard in their own "Subsystems" tab. */
    Shuffleboard.getTab("Subsystems").add("Swerve Drive", swerveDrive);
    Shuffleboard.getTab("Subsystems").add("Indexing Subsystem", indexer);
    Shuffleboard.getTab("Subsystems").add("Declination Subsystem", declination);
    Shuffleboard.getTab("Subsystems").add("Pneumatics System", pneumatics);
    Shuffleboard.getTab("Subsystems").add("Right Ascension Subsystem", rightAscension);
    Shuffleboard.getTab("Subsystems").add("Aim Simulator", aimer);

    /* Put the Pose Estimators on Dashboards */
    SmartDashboard.putData("Field", m_field_poseestimator);
    // Shuffleboard.getTab("Tab 6").add(m_field_poseestimator);
  }

  public void updateDashboard() {

    SmartDashboard.putNumber("X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber(
        "Rads", poseEstimator.getEstimatedPosition().getRotation().getRadians());

    SmartDashboard.putNumber("Angle", getRotation2d().getDegrees());
  }

  public GenericEntry yActual, yGoal, xActual, xGoal, barrelActual, barrelGoal;
  // ** NetworkTableEntry for the encoders of the turn motors */
  private GenericEntry abs_Enc_FR_Offset_Entry,
      abs_Enc_FL_Offset_Entry,
      abs_Enc_BR_Offset_Entry,
      abs_Enc_BL_Offset_Entry;
  private GenericEntry abs_Enc_FR_Raw_Entry,
      abs_Enc_FL_Raw_Entry,
      abs_Enc_BR_Raw_Entry,
      abs_Enc_BL_Raw_Entry;
  private GenericEntry enc_FR_pos_Entry, enc_FL_pos_Entry, enc_BR_pos_Entry, enc_BL_pos_Entry;
  // private GenericEntry enc_FR_vel_Entry, enc_FL_vel_Entry, enc_BR_vel_Entry,
  // enc_BL_vel_Entry;

  // ShuffleBoardLayouts for putting encoders onto the board
  private ShuffleboardLayout absolute_encoders_offset_list =
      Shuffleboard.getTab("Encoders")
          .getLayout("Absolute Encoders Offset", BuiltInLayouts.kGrid)
          .withSize(2, 2);

  private ShuffleboardLayout absolute_encoders_no_offset_list =
      Shuffleboard.getTab("Encoders")
          .getLayout("Absolute Encoders No Offset", BuiltInLayouts.kGrid)
          .withSize(2, 2);
  private ShuffleboardLayout turn_encoders_positions =
      Shuffleboard.getTab("Encoders")
          .getLayout("Turn Encoders Position(Rad)", BuiltInLayouts.kGrid)
          .withSize(2, 2);

  public void moduleEncodersInit(SwerveDrive swerveDrive
      // IndexingSubsystem indexer,
      // DeclinationSubsystem declination,
      // PneumaticsSystem pneumatics,
      // RightAscensionSubsystem rightAscension,
      // AimSimulator aimer
      ) {
    abs_Enc_FR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.frontRight.getName(),
                swerveDrive.frontRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_FL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.frontLeft.getName(),
                swerveDrive.frontLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.backRight.getName(),
                swerveDrive.backRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.backLeft.getName(),
                swerveDrive.backLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();

    enc_FR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.frontRight.getName(), swerveDrive.frontRight.getTurnPosition())
            .getEntry();
    enc_FL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.frontLeft.getName(), swerveDrive.frontLeft.getTurnPosition())
            .getEntry();
    enc_BR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.backRight.getName(), swerveDrive.backRight.getTurnPosition())
            .getEntry();
    enc_BL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.backLeft.getName(), swerveDrive.backLeft.getTurnPosition())
            .getEntry();

    abs_Enc_FR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.frontRight.getName(),
                swerveDrive.frontRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_FL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.frontLeft.getName(),
                swerveDrive.frontLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.backRight.getName(),
                swerveDrive.backRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.backLeft.getName(),
                swerveDrive.backLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
  }

  public void aimerShuffleBoardInit(
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      AimSimulator aimer) {
    yActual =
        Shuffleboard.getTab("Aimer")
            .add("Y Actual", declination.declination1.getPosition())
            .getEntry();

    yGoal = Shuffleboard.getTab("Aimer").add("Y Goal", declination.getGoal().position).getEntry();

    xActual =
        Shuffleboard.getTab("Aimer")
            .add("X Actual", rightAscension.rightAscensionMotor.getPosition())
            .getEntry();

    xGoal =
        Shuffleboard.getTab("Aimer").add("X Goal", rightAscension.getGoal().position).getEntry();

    barrelActual =
        Shuffleboard.getTab("Aimer")
            .add("Actual Barrel Angle", indexer.indexMotor.getPosition())
            .getEntry();

    barrelGoal =
        Shuffleboard.getTab("Aimer").add("Goal Barrel Angle", aimer.barrelAngle).getEntry();
  }

  public void updateModuleEncoders(SwerveDrive swerveDrive) {
    abs_Enc_FR_Offset_Entry.setDouble(swerveDrive.frontRight.getAbsoluteEncoderRadiansOffset());
    abs_Enc_FL_Offset_Entry.setDouble(swerveDrive.frontLeft.getAbsoluteEncoderRadiansOffset());
    abs_Enc_BR_Offset_Entry.setDouble(swerveDrive.backRight.getAbsoluteEncoderRadiansOffset());
    abs_Enc_BL_Offset_Entry.setDouble(swerveDrive.backLeft.getAbsoluteEncoderRadiansOffset());

    abs_Enc_FR_Raw_Entry.setDouble(swerveDrive.frontRight.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_FL_Raw_Entry.setDouble(swerveDrive.frontLeft.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_BR_Raw_Entry.setDouble(swerveDrive.backRight.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_BL_Raw_Entry.setDouble(swerveDrive.backLeft.getAbsoluteEncoderRadiansNoOffset());

    enc_FR_pos_Entry.setDouble(swerveDrive.frontRight.getTurnPosition());
    enc_FL_pos_Entry.setDouble(swerveDrive.frontLeft.getTurnPosition());
    enc_BR_pos_Entry.setDouble(swerveDrive.backRight.getTurnPosition());
    enc_BL_pos_Entry.setDouble(swerveDrive.backLeft.getTurnPosition());
  }

  /** Pose Helper Methods */

  /**
   * Gets the position of the robot in Pose2d format. Uses odometer reading. Includes the x, y, and
   * theta values of the robot.
   *
   * @return The Pose2d of the robot.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void printPos2d() {
    System.out.println(poseEstimator.getEstimatedPosition());
  }

  public Rotation2d getAngleBetweenCurrentAndTargetPose(Pose2d targetPose) {
    Rotation2d targetYaw = PhotonUtils.getYawToPose(getPose(), targetPose);
    return targetYaw;
  }

  /** Odometry */

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   */
  public void setOdometry() {
    poseEstimator.resetPosition(getRotation2d(), SwerveDrive.swerveModulePositions, getPose());
  }

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   *
   * @param pos the Pose2d to set the odometry
   */
  public void setOdometry(Pose2d pos) {
    poseEstimator.resetPosition(getRotation2d(), SwerveDrive.swerveModulePositions, pos);
  }
  /** Gyroscope Methods (NavX) */
  public void zeroHeading() {
    gyro.reset();
  }

  public double getPitch() {
    return gyro.getPitch() - 1.14;
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  /**
   * Method to get the facing direction of the gyro.
   *
   * @return The facing direction of the gyro, between -360 and 360 degrees.
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw() + 180, 360);
  }

  /**
   * Gets the Rotation2d value of the facing direction of the robot.
   *
   * @return The facing direction of the robot in Rotation2d format.
   */
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
    // .plus(Rotation2d.fromRadians(Math.PI));
  }

  public RobotState() {

    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
            })
        .start();
  }
}
