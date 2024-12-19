// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.helpers.Vision.VisionData;
import frc.maps.Constants;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

/** RobotState is used to retrieve information about the robot's state in other classes. */
public class RobotState {

  public static RobotState instance;
  // Initialize gyro
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public final Field2d gameField = new Field2d();

  /** Technically pose ESTIMATES */
  public Pose2d lastPose = new Pose2d();

  public Pose2d currentPose = new Pose2d();

  // SwerveDriveOdometry odometer;
  public SwerveDrivePoseEstimator poseEstimator;
  public PhotonPoseEstimator photonPoseEstimator;

  public final VisionData visionData = new VisionData();

  public void poseInit() {

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.getAngle()).unaryMinus(),
            SwerveDrive.getInstance().swerveModulePositions,
            new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void updatePose() {

    gameField.setRobotPose(getPose());
    Logger.recordOutput("REAL Pose", getPose());

    lastPose = currentPose;

    /** Update the visionData to what the camera sees. */
    PhotonVision.getInstance().updateData(visionData, getPose());

    for (int i = 0; i < visionData.poseEstimates.size(); i++) {
      /** Add the Photonvision pose estimates */
      poseEstimator.addVisionMeasurement(visionData.poseEstimates.get(i), visionData.timestamp);
    }

    /** Update the SwerveDrivePoseEstimator with the Drivetrain encoders and such */
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), getRotation2d(), SwerveDrive.getInstance().swerveModulePositions);

    currentPose = getPose();
  }

  public void dashboardInit() {

    /* Put the Command Scheduler on SmartDashboard */
    SmartDashboard.putData(CommandScheduler.getInstance());

    /* Put all the subsystems on ShuffleBoard in their own "Subsystems" tab. */
    Shuffleboard.getTab("Subsystems").add("Swerve Drive", SwerveDrive.getInstance());
    Shuffleboard.getTab("Subsystems").add("Indexing Subsystem", IndexingSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems")
        .add("Declination Subsystem", DeclinationSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Pneumatics System", PneumaticsSystem.getInstance());
    Shuffleboard.getTab("Subsystems")
        .add("Right Ascension Subsystem", RightAscensionSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Aim Simulator", AimSimulator.getInstance());
    Shuffleboard.getTab("Subsystems").add("Lights", Lights.getInstance());

    /* Put the Pose Estimators on Dashboards */
    SmartDashboard.putData("Field", gameField);
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

  public void moduleEncodersInit(SwerveDrive swerveDrive) {

    // IndexingSubsystem indexer,
    // DeclinationSubsystem declination,
    // PneumaticsSystem pneumatics,

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

    enc_FR_pos_Entry.setDouble(SwerveDrive.getInstance().frontRight.getTurnPosition());
    enc_FL_pos_Entry.setDouble(SwerveDrive.getInstance().frontLeft.getTurnPosition());
    enc_BR_pos_Entry.setDouble(SwerveDrive.getInstance().backRight.getTurnPosition());
    enc_BL_pos_Entry.setDouble(SwerveDrive.getInstance().backLeft.getTurnPosition());
  }

  public void updateModulePositions() {
    SwerveDrive.getInstance().swerveModulePositions[0] =
        new SwerveModulePosition(
            SwerveDrive.getInstance().frontRight.getDrivePosition(),
            new Rotation2d(SwerveDrive.getInstance().frontRight.getTurnPosition()));
    SwerveDrive.getInstance().swerveModulePositions[1] =
        new SwerveModulePosition(
            SwerveDrive.getInstance().frontLeft.getDrivePosition(),
            new Rotation2d(SwerveDrive.getInstance().frontLeft.getTurnPosition()));
    SwerveDrive.getInstance().swerveModulePositions[2] =
        new SwerveModulePosition(
            SwerveDrive.getInstance().backRight.getDrivePosition(),
            new Rotation2d(SwerveDrive.getInstance().backRight.getTurnPosition()));
    SwerveDrive.getInstance().swerveModulePositions[3] =
        new SwerveModulePosition(
            SwerveDrive.getInstance().backLeft.getDrivePosition(),
            new Rotation2d(SwerveDrive.getInstance().backLeft.getTurnPosition()));
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

  // /** If Using REFERENCE POSE STRAT */
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  //   photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  //   return photonPoseEstimator.update();
  // }

  public void printPos2d() {
    System.out.println(poseEstimator.getEstimatedPosition());
  }

  // Returns the estimated transformation over the next tick (The change in
  // position)
  private Transform2d getTickFutureTransform() {
    return new Transform2d(
        new Translation2d(
            SwerveDrive.getInstance().getFieldVelocity().vxMetersPerSecond * 0.02,
            SwerveDrive.getInstance().getFieldVelocity().vyMetersPerSecond * 0.02),
        new Rotation2d(0.0));
  }

  // Returns the estimated robot position a tick from the current time (Theoretically?)
  private Pose2d getFutureTickPose() {
    return getPose().plus(getTickFutureTransform().inverse());
  }

  /** Can use this for turret. */
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
    poseEstimator.resetPosition(
        getRotation2d(), SwerveDrive.getInstance().swerveModulePositions, getPose());
  }

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   *
   * @param pos the Pose2d to set the odometry
   */
  public void setOdometry(Pose2d pos) {
    poseEstimator.resetPosition(
        getRotation2d(), SwerveDrive.getInstance().swerveModulePositions, pos);
  }
  /** Gyroscope Methods (NavX) */
  public void zeroHeading() {
    gyro.reset();
    setOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0)));
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
    return Math.IEEEremainder(gyro.getYaw(), 360);
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

  private RobotState() {

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

  public void periodic() {}
}
