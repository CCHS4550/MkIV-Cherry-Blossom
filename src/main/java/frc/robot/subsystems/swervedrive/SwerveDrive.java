package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;


/** Class for controlling a swerve drive chassis. Consists of 4 SwerveModules and a gyro. */
public class SwerveDrive extends SubsystemBase {

  // Initializing swerve modules. Must include full CCSparkMax object
  // declarations.

  public final SwerveModule frontRight =
      new SwerveModule(
          new CCSparkMax(
              "Front Right Drive",
              "frd",
              Constants.MotorConstants.FRONT_RIGHT_DRIVE,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.FRONT_RIGHT_DRIVE_REVERSE,
              Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
              Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
          new CCSparkMax(
              "Front Right Turn",
              "frt",
              Constants.MotorConstants.FRONT_RIGHT_TURN,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.FRONT_RIGHT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
          "Front Right");

  public final SwerveModule frontLeft =
      new SwerveModule(
          new CCSparkMax(
              "Front Left Drive",
              "fld",
              Constants.MotorConstants.FRONT_LEFT_DRIVE,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.FRONT_LEFT_DRIVE_REVERSE,
              Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
              Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
          new CCSparkMax(
              "Front Left Turn",
              "flt",
              Constants.MotorConstants.FRONT_LEFT_TURN,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.FRONT_LEFT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
          "Front Left");

  public final SwerveModule backRight =
      new SwerveModule(
          new CCSparkMax(
              "Back Right Drive",
              "brd",
              Constants.MotorConstants.BACK_RIGHT_DRIVE,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.BACK_RIGHT_DRIVE_REVERSE,
              Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
              Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
          new CCSparkMax(
              "Back Right Turn",
              "brt",
              Constants.MotorConstants.BACK_RIGHT_TURN,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.BACK_RIGHT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
          "Back Right");

  public final SwerveModule backLeft =
      new SwerveModule(
          new CCSparkMax(
              "Back Left Drive",
              "bld",
              Constants.MotorConstants.BACK_LEFT_DRIVE,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.BACK_LEFT_DRIVE_REVERSE,
              Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
              Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
          new CCSparkMax(
              "Back Left Turn",
              "blt",
              Constants.MotorConstants.BACK_LEFT_TURN,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
          "Back Left");

  // * Must be in the order FR, FL, BR, BL */
  private SwerveModule[] swerveModules =
      new SwerveModule[] {frontRight, frontLeft, backRight, backLeft};

  /** Module positions used for odometry */
  public static SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

  
  /* PID Controllers */
  public PIDController xPID, yPID;
  public PIDController turnPID;
  ProfiledPIDController turnPIDProfiled;
  // ProfiledPIDController turnPID;

  


  private final Unit<Velocity<Voltage>> VoltsPerSecond = Volts.per(Second);

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              VoltsPerSecond.of(1),
              Volts.of(3),
              Seconds.of(3),
              (state) ->
                  org.littletonrobotics.junction.Logger.recordOutput(
                      "SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setDriveVoltages(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  public Rotation2d initialAngle = new Rotation2d(0);

  public Pose2d[] speakerPoses = new Pose2d[3];
  /**
   * Creates a new SwerveDrive object. Delays 1 second before setting gyro to 0 to account for gyro
   * calibration time.
   */
  public void getAbsoluteEncoderoffsets() {

    System.out.println("frontRight:" + frontRight.getAbsoluteEncoderRadiansNoOffset());
    System.out.println("frontLeft:" + frontLeft.getAbsoluteEncoderRadiansNoOffset());
    System.out.println("backRight:" + backRight.getAbsoluteEncoderRadiansNoOffset());
    System.out.println("backLeft:" + backLeft.getAbsoluteEncoderRadiansNoOffset());
  }

  /** Constructor for the Swerve Drive Subsystem.  */
  public SwerveDrive() {
    swerveModulePositions[0] =
        new SwerveModulePosition(0, new Rotation2d(frontRight.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[1] =
        new SwerveModulePosition(0, new Rotation2d(frontLeft.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[2] =
        new SwerveModulePosition(0, new Rotation2d(backRight.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[3] =
        new SwerveModulePosition(0, new Rotation2d(backLeft.getAbsoluteEncoderRadiansOffset()));

    

    xPID = new PIDController(1, .1, 0);
    yPID = new PIDController(1, .1, 0);
    // xPID = new PIDController(1, 0, 0);
    // yPID = new PIDController(1, 0, 0);

    // *TODO: Possibly research profiled PID
    // turnPID = new ProfiledPIDController(0.5, 0, 0,
    // RobotMap.thetaControllConstraints);
    turnPID = new PIDController(0.3, 0, 0);
    turnPIDProfiled =
        new ProfiledPIDController(
            .7,
            0,
            0,
            new Constraints(
                Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                Constants.SwerveConstants.TURN_RATE_LIMIT));
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    RobotState.getInstance().moduleEncodersInit(this);

  }

  /**
   * Resets chassis gyro to 0. For all gyro purposes, "heading" refers to the facing direction of
   * the gyro.
   */
  public void spinMotor(double speed) {
    frontRight.setDriveVelocity(speed);
    frontRight.setTurnPosition(() -> speed);
  }



  /** Returns the nearest speaker pose for for alliance color */
  @Override
  public void periodic() {

    // m_field.setRobotPose(poseEstimator.getEstimatedPosition());

    Logger.recordOutput("Real moduleStates", getCurrentModuleStates());
    Logger.recordOutput("Angle Rotation2d", RobotState.getInstance().getRotation2d());

  
    RobotState.getInstance().updateModuleEncoders(this);
    

    
    updateModulePositions();

    
  }

  /** Sets all 4 modules' drive and turn speeds to 0. */
  public void stopModules() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, new Rotation2d(frontRight.getAbsoluteEncoderRadiansOffset())),
          new SwerveModuleState(0, new Rotation2d(frontLeft.getAbsoluteEncoderRadiansOffset())),
          new SwerveModuleState(0, new Rotation2d(backRight.getAbsoluteEncoderRadiansOffset())),
          new SwerveModuleState(0, new Rotation2d(backLeft.getAbsoluteEncoderRadiansOffset()))
        };
    setModuleStates(states);
  }

  /**
   * Sets all 4 modules' drive and turn speeds with the SwerveModuleState format.
   *
   * @param desiredStates The array of the states that each module will be set to in the
   *     SwerveModuleState format.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // currentSwerveModuleStates = desiredStates;
    boolean openLoop = false;
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     desiredStates, Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL);
    // Logger.recordOutput("SwerveModuleStates/SetpointsOptimized", desiredStates);
    frontRight.setDesiredState(desiredStates[0], openLoop);
    frontLeft.setDesiredState(desiredStates[1], openLoop);
    backRight.setDesiredState(desiredStates[2], openLoop);
    backLeft.setDesiredState(desiredStates[3], openLoop);
  }

  /* Returns the actual moduleStates */
  public SwerveModuleState[] getCurrentModuleStates() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          frontRight.getState(), frontLeft.getState(), backRight.getState(), backLeft.getState()
        };
    return states;
  }



  public void updateModulePositions() {
    swerveModulePositions[0] =
        new SwerveModulePosition(
            frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition()));
    swerveModulePositions[1] =
        new SwerveModulePosition(
            frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition()));
    swerveModulePositions[2] =
        new SwerveModulePosition(
            backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition()));
    swerveModulePositions[3] =
        new SwerveModulePosition(
            backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition()));
  }


  
  /*
   * Used for Autobuilder in AutonomousScheme.java
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getCurrentModuleStates()),
        RobotState.getInstance().getRotation2d());
  }

  /*
   * Used for Autobuilder in AutonomousScheme.java
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    Logger.recordOutput("Autonomous Set moduleStates", moduleStates);
    setModuleStates(moduleStates);
  }




  public void printWorld() {
    System.out.println("Hello World!");
  }



  /* SysID Factory Methods */
  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the dynamic characterization test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Used only in Characterizing. Don't touch this. Sets the provided voltages and locks the wheels
   * to 0 radians.
   *
   * @param volts
   */
  public void setDriveVoltages(Measure<Voltage> volts) {
    for (SwerveModule s : swerveModules) {
      s.setTurnPosition(() -> 0);
      s.setDriveVoltage(volts.in(Volts));
    }
  }

  public void test(double driveSpeed, double turnSpeed) {
    backRight.driveAndTurn(driveSpeed, turnSpeed);
    backRight.printEncoders();
  }

  public void setRawDriveVolts(double volt) {
    frontRight.setDriveVoltage(volt);
    frontLeft.setDriveVoltage(volt);
    backRight.setDriveVoltage(volt);
    backLeft.setDriveVoltage(volt);

    frontRight.setTurnPosition(() -> Math.PI / 2);
    frontLeft.setTurnPosition(() -> Math.PI / 2);
    backRight.setTurnPosition(() -> Math.PI / 2);
    backLeft.setTurnPosition(() -> Math.PI / 2);
  }


  public Command halt() {
    return Commands.runOnce(() -> {}, this);
  }

  public void setspeeds(double speed) {
    frontRight.setDriveVelocity(speed);
    frontLeft.setDriveVelocity(speed);
    backRight.setDriveVelocity(speed);
    backLeft.setDriveVelocity(speed);
  }

  public void setInitialAngle() {}

}

