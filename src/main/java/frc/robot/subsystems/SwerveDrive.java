package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;

// import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

/**
 * Class for controlling a swerve drive chassis. Consists of 4 SwerveModules and
 * a gyro.
 */
public class SwerveDrive extends SubsystemBase {

        private boolean test = false;

        // Initializing swerve modules. Must include full CCSparkMax object
        // declarations.
        private final Field2d m_field_poseestimator= new Field2d();
        private final Field2d m_field_getPose= new Field2d();
        public final SwerveModule frontRight = new SwerveModule(
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

        public final SwerveModule frontLeft = new SwerveModule(
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

        public final SwerveModule backRight = new SwerveModule(
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

        public final SwerveModule backLeft = new SwerveModule(
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
        private SwerveModule[] swerveModules = new SwerveModule[] { frontRight, frontLeft, backRight, backLeft };
        // Initialize gyro
        private AHRS gyro = new AHRS(SPI.Port.kMXP);

        // SwerveDriveOdometry odometer;
        SwerveDrivePoseEstimator poseEstimator;
        PhotonPoseEstimator photonPoseEstimator;
        PIDController xPID, yPID;
        public PIDController turnPID;
        ProfiledPIDController turnPIDProfiled;
        // ProfiledPIDController turnPID;

        /** Module positions used for odometry */
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

        // ** NetworkTableEntry for the encoders of the turn motors */
        private GenericEntry abs_Enc_FR_Offset_Entry, abs_Enc_FL_Offset_Entry, abs_Enc_BR_Offset_Entry,
                        abs_Enc_BL_Offset_Entry;
        private GenericEntry abs_Enc_FR_Raw_Entry, abs_Enc_FL_Raw_Entry, abs_Enc_BR_Raw_Entry, abs_Enc_BL_Raw_Entry;

        private GenericEntry enc_FR_pos_Entry, enc_FL_pos_Entry, enc_BR_pos_Entry, enc_BL_pos_Entry;
        // private GenericEntry enc_FR_vel_Entry, enc_FL_vel_Entry, enc_BR_vel_Entry,
        // enc_BL_vel_Entry;

        private final Unit<Velocity<Voltage>> VoltsPerSecond = Volts.per(Second);

        // ShuffleBoardLayouts for putting encoders onto the board
        private ShuffleboardLayout absolute_encoders_offset_list = Shuffleboard.getTab("Encoders")
                        .getLayout("Absolute Encoders Offset", BuiltInLayouts.kGrid).withSize(2, 2);

        private ShuffleboardLayout absolute_encoders_no_offset_list = Shuffleboard.getTab("Encoders")
                        .getLayout("Absolute Encoders No Offset", BuiltInLayouts.kGrid).withSize(2, 2);
        private ShuffleboardLayout turn_encoders_positions = Shuffleboard.getTab("Encoders")
                        .getLayout("Turn Encoders Position(Rad)", BuiltInLayouts.kGrid)
                        .withSize(2, 2);

        // SysIdRoutine sysIdRoutine = new SysIdRoutine(
        //                 new SysIdRoutine.Config(VoltsPerSecond.of(1), Volts.of(5), Seconds.of(5),
        //                                 (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        //                 new SysIdRoutine.Mechanism(
        //                                 (voltage) -> setDriveVoltages(voltage),
        //                                 null, // No log consumer, since data is recorded by URCL
        //                                 this));




        public Rotation2d initialAngle = new Rotation2d(0);

        public Pose2d[] speakerPoses = new Pose2d[3];
        /**
         * Creates a new SwerveDrive object. Delays 1 second before setting gyro to 0 to
         * account for gyro calibration time.
         */

         public void getAbsoluteEncoderoffsets(){

         System.out.println("frontRight:" + frontRight.getAbsoluteEncoderRadiansNoOffset());
         System.out.println("frontLeft:" + frontLeft.getAbsoluteEncoderRadiansNoOffset());
         System.out.println("backRight:" + backRight.getAbsoluteEncoderRadiansNoOffset());
         System.out.println("backLeft:" + backLeft.getAbsoluteEncoderRadiansNoOffset());

         }


        public SwerveDrive() {
                swerveModulePositions[0] = new SwerveModulePosition(0,
                                new Rotation2d(frontRight.getAbsoluteEncoderRadiansOffset()));
                swerveModulePositions[1] = new SwerveModulePosition(0,
                                new Rotation2d(frontLeft.getAbsoluteEncoderRadiansOffset()));
                swerveModulePositions[2] = new SwerveModulePosition(0,
                                new Rotation2d(backRight.getAbsoluteEncoderRadiansOffset()));
                swerveModulePositions[3] = new SwerveModulePosition(0,
                                new Rotation2d(backLeft.getAbsoluteEncoderRadiansOffset()));

                poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DRIVE_KINEMATICS,
                                Rotation2d.fromDegrees(gyro.getAngle()).unaryMinus(), swerveModulePositions, new Pose2d(0, 0, new Rotation2d(0)));

                xPID = new PIDController(1, .1, 0);
                yPID = new PIDController(1, .1, 0);
                // xPID = new PIDController(1, 0, 0);
                // yPID = new PIDController(1, 0, 0);

                // *TODO: Possibly research profiled PID
                // turnPID = new ProfiledPIDController(0.5, 0, 0,
                // RobotMap.thetaControllConstraints);
                turnPID = new PIDController(0.3, 0, 0);
                turnPIDProfiled = new ProfiledPIDController(.7, 0, 0, new Constraints(
                                Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                Constants.SwerveConstants.TURN_RATE_LIMIT));
                turnPID.enableContinuousInput(-Math.PI, Math.PI);

                initShuffleBoardEncoders();

                SmartDashboard.putData("Field", m_field_poseestimator);
                Shuffleboard.getTab("Tab 6").add(m_field_poseestimator);
                new Thread(() -> {
                        try {
                                Thread.sleep(1000);
                                zeroHeading();
                        } catch (Exception e) {
                        }
                }).start();
            }

                /**
                 * The autobuilder for Path Planner Autos
                 */
        //         AutoBuilder.configureHolonomic(
        //                         this::getPose, // Robot pose supplier
        //                         this::setOdometry, // Method to reset odometry (will be called if your auto has a
        //                                            // starting pose)
        //                         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //                         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        //                                                   // ChassisSpeeds
        //                         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
        //                                                          // in your Constants class
        //                                         new PIDConstants(1, 0, 0.0), // Translation PID constants
        //                                         new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
        //                                         Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL, // Max
        //                                                                                                                  // module
        //                                                                                                                  // speed,
        //                                                                                                                  // in
        //                                                                                                                  // m/s
        //                                         0.43105, // Drive base radius in meters. Distance from robot center to
        //                                                  // furthest module.
        //                                         new ReplanningConfig() // Default path replanning config. See the API
        //                                                                // for the options here
        //                         ),
        //                         () -> {
        //                                 // Boolean supplier that controls when the path will be mirrored for the red
        //                                 // alliance
        //                                 // This will flip the path being followed to the red side of the field.
        //                                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //                                 var alliance = DriverStation.getAlliance();
        //                                 if (alliance.isPresent()) {
        //                                         return alliance.get() == DriverStation.Alliance.Red;
        //                                 }
        //                                 return false;
        //                         },
        //                         this // Reference to this subsystem to set requirements
        //         );
        //         if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
        //                 speakerPoses = Constants.RedFieldPositionConstants.SPEAKER_POSES;
        //         } else {
        //                 speakerPoses = Constants.BlueFieldPositionConstants.SPEAKER_POSES;
        //         }
        // }

        /**
         * Resets chassis gyro to 0. For all gyro purposes, "heading" refers to the
         * facing direction of the gyro.
         */

        public void spinMotor(double speed) {
                frontRight.setDriveVelocity(speed);
                frontRight.setTurnPosition(() -> speed);
        }

        public void zeroHeading() {
                gyro.reset();
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
                return gyro.getRotation2d().plus(Rotation2d.fromRadians(Math.PI));
        }

        /**
         * Returns the nearest speaker pose for for alliance color
         * 
         */
        @Override
        public void periodic() {
                

                // m_field.setRobotPose(poseEstimator.getEstimatedPosition());
                // Logger.recordOutput("SwerveModuleStates/MeasuredOutputs", getCurrentModuleStates());

                poseEstimator.update(getRotation2d(), swerveModulePositions);

                updateShuffleBoardEncoders();

                updateOdometer();

                m_field_poseestimator.setRobotPose(poseEstimator.getEstimatedPosition());
                m_field_getPose.setRobotPose(getPose());
                
                SmartDashboard.putNumber("X", poseEstimator.getEstimatedPosition().getX());
                SmartDashboard.putNumber("Y", poseEstimator.getEstimatedPosition().getY());
                SmartDashboard.putNumber("Rads", poseEstimator.getEstimatedPosition().getRotation().getRadians());

                // SmartDashboard.putNumber("pose to middle", getPose().getTranslation().getDistance(speakerPoses[0].getTranslation()));
                // SmartDashboard.putBoolean("inAmpPosition", MechanismScheme.inAmpPosition.getAsBoolean());
                // SmartDashboard.putNumber("Distance from Tag 4", Constants.cameraOne.FRONT_CAMERA.)
                // MechanismScheme.periodic();
        }

        /**
         * Sets all 4 modules' drive and turn speeds to 0.
         */
        public void stopModules() {
                SwerveModuleState[] states = new SwerveModuleState[] {
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
         * @param desiredStates The array of the states that each module will be set to
         *                      in the SwerveModuleState format.
         */
        public void setModuleStates(SwerveModuleState[] desiredStates) {
                // currentSwerveModuleStates = desiredStates;
                boolean openLoop = false;
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                                Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL);
                // Logger.recordOutput("SwerveModuleStates/SetpointsOptimized", desiredStates);
                frontRight.setDesiredState(desiredStates[0], openLoop);
                frontLeft.setDesiredState(desiredStates[1], openLoop);
                backRight.setDesiredState(desiredStates[2], openLoop);
                backLeft.setDesiredState(desiredStates[3], openLoop);
        }

        public SwerveModuleState[] getCurrentModuleStates() {
                SwerveModuleState[] states = new SwerveModuleState[] {
                                frontRight.getState(), frontLeft.getState(), backRight.getState(),
                                backLeft.getState() };
                return states;
        }

        /**
         * Resets the odometer readings using the gyro, SwerveModulePositions (defined
         * in constructor), and Pose2d.
         */
        public void setOdometry() {
                poseEstimator.resetPosition(getRotation2d(), swerveModulePositions, getPose());
        }

        /**
         * Resets the odometer readings using the gyro, SwerveModulePositions (defined
         * in constructor), and Pose2d.
         * 
         * @param pos the Pose2d to set the odometry
         */
        public void setOdometry(Pose2d pos) {
                poseEstimator.resetPosition(getRotation2d(), swerveModulePositions, pos);
        }

        public void updateModulePositions() {
                swerveModulePositions[0] = new SwerveModulePosition(frontRight.getDrivePosition(),
                                new Rotation2d(frontRight.getTurnPosition()));
                swerveModulePositions[1] = new SwerveModulePosition(frontLeft.getDrivePosition(),
                                new Rotation2d(frontLeft.getTurnPosition()));
                swerveModulePositions[2] = new SwerveModulePosition(backRight.getDrivePosition(),
                                new Rotation2d(backRight.getTurnPosition()));
                swerveModulePositions[3] = new SwerveModulePosition(backLeft.getDrivePosition(),
                                new Rotation2d(backLeft.getTurnPosition()));
        }

        /**
         * Gets the position of the robot in Pose2d format. Uses odometer reading.
         * Includes the x, y, and theta values of the robot.
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

        public void updateOdometer() {
                updateModulePositions();

                SmartDashboard.putNumber("Angle", getRotation2d().getDegrees());
                poseEstimator.update(getRotation2d(), swerveModulePositions);

                // if (estimatedPoseOptional.isPresent()) {
                // EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
                // poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                // estimatedRobotPose.timestampSeconds,
                // Constants.cameraOne.FRONT_CAMERA.getEstimationStdDevs(
                // estimatedRobotPose.estimatedPose.toPose2d()));
                // }

                // Should do the same thing as above x
                
                // Optional<EstimatedRobotPose> estimatedPoseOptional = Constants.cameraOne.FRONT_CAMERA
                //                 .getEstimatedGlobalPose(getPose());
                // estimatedPoseOptional.ifPresent(est -> {
                //          var estPose = est.estimatedPose.toPose2d();
                //         // Change our trust in the measurement based on the tags we can see
                //         // var estStdDevs = Constants.cameraOne.FRONT_CAMERA.getEstimationStdDevs(
                //         //                 estPose);

                //         poseEstimator.addVisionMeasurement(
                //                         estPose, est.timestampSeconds
                //                         // , estStdDevs
                //                         );
               
                // });


                // Logger.recordOutput("Odometry/Pose2D", poseEstimator.getEstimatedPosition());
                
        }
        // public double getAutoWristAngle(){
        //          int tagId = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
        //         Pose2d middleSubwoofer;
        //         if (tagId ==4){

        //                  middleSubwoofer = RedFieldPositionConstants.SPEAKER_FRONT;
                
        //         }
        //         else{
        //                 middleSubwoofer = BlueFieldPositionConstants.SPEAKER_FRONT;
                
        //         }
        //         //   double distanceFromMiddleSubwoofer = PhotonUtils.getDistanceToPose(estPose, middleSubwoofer);
        //         //   double wristAngleToShoot = 15.619040489196777*(1- (distanceFromMiddleSubwoofer/Units.feetToMeters(1)));
        // }
        // }
        public ChassisSpeeds getRobotRelativeSpeeds() {
                return ChassisSpeeds.fromRobotRelativeSpeeds(
                                Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getCurrentModuleStates()),
                                getRotation2d());
        }

        public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
                SwerveModuleState[] moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                setModuleStates(moduleStates);
        }

        /**
         * Default end state of 0 mps and 0 degrees
         * 
         * @param poses an array of poses to have on the path
         * @return A PathPlannerPath following given poses
         */
        public PathPlannerPath onTheFlyPath(Pose2d[] poses) {
                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

                // Create the path using the bezier points created above
                PathPlannerPath path = new PathPlannerPath(
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
         * 
         * @param poses           an array of poses to have on the path
         * @param desiredEndState the desired end state of the path in mps and degrees
         * @return an on the fly PathPlannerPath
         */
        public PathPlannerPath onTheFlyPathFromPoses(Pose2d[] poses, GoalEndState desiredEndState) {
                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

                // Create the path using the bezier points created above
                PathPlannerPath path = new PathPlannerPath(
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

        public Rotation2d getAngleBetweenCurrentAndTargetPose(Pose2d targetPose) {
                Rotation2d targetYaw = PhotonUtils.getYawToPose(getPose(), targetPose);
                return targetYaw;
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
                Command pathfindingCommand = AutoBuilder.pathfindToPose(
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
                return AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
                                0);
        }

        public void initShuffleBoardEncoders() {
                abs_Enc_FR_Offset_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_offset_list.getTitle())
                                .add(frontRight.getName(), frontRight.getAbsoluteEncoderRadiansOffset()).getEntry();
                abs_Enc_FL_Offset_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_offset_list.getTitle())
                                .add(frontLeft.getName(), frontLeft.getAbsoluteEncoderRadiansOffset()).getEntry();
                abs_Enc_BR_Offset_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_offset_list.getTitle())
                                .add(backRight.getName(), backRight.getAbsoluteEncoderRadiansOffset()).getEntry();
                abs_Enc_BL_Offset_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_offset_list.getTitle())
                                .add(backLeft.getName(), backLeft.getAbsoluteEncoderRadiansOffset()).getEntry();

                enc_FR_pos_Entry = Shuffleboard.getTab("Encoders").getLayout(turn_encoders_positions.getTitle())
                                .add(frontRight.getName(), frontRight.getTurnPosition()).getEntry();
                enc_FL_pos_Entry = Shuffleboard.getTab("Encoders").getLayout(turn_encoders_positions.getTitle())
                                .add(frontLeft.getName(), frontLeft.getTurnPosition()).getEntry();
                enc_BR_pos_Entry = Shuffleboard.getTab("Encoders").getLayout(turn_encoders_positions.getTitle())
                                .add(backRight.getName(), backRight.getTurnPosition()).getEntry();
                enc_BL_pos_Entry = Shuffleboard.getTab("Encoders").getLayout(turn_encoders_positions.getTitle())
                                .add(backLeft.getName(), backLeft.getTurnPosition()).getEntry();

                abs_Enc_FR_Raw_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_no_offset_list.getTitle())
                                .add(frontRight.getName(), frontRight.getAbsoluteEncoderRadiansNoOffset()).getEntry();
                abs_Enc_FL_Raw_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_no_offset_list.getTitle())
                                .add(frontLeft.getName(), frontLeft.getAbsoluteEncoderRadiansNoOffset()).getEntry();
                abs_Enc_BR_Raw_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_no_offset_list.getTitle())
                                .add(backRight.getName(), backRight.getAbsoluteEncoderRadiansNoOffset()).getEntry();
                abs_Enc_BL_Raw_Entry = Shuffleboard.getTab("Encoders")
                                .getLayout(absolute_encoders_no_offset_list.getTitle())
                                .add(backLeft.getName(), backLeft.getAbsoluteEncoderRadiansNoOffset()).getEntry();

        }

        public void updateShuffleBoardEncoders() {
                abs_Enc_FR_Offset_Entry.setDouble(frontRight.getAbsoluteEncoderRadiansOffset());
                abs_Enc_FL_Offset_Entry.setDouble(frontLeft.getAbsoluteEncoderRadiansOffset());
                abs_Enc_BR_Offset_Entry.setDouble(backRight.getAbsoluteEncoderRadiansOffset());
                abs_Enc_BL_Offset_Entry.setDouble(backLeft.getAbsoluteEncoderRadiansOffset());

                abs_Enc_FR_Raw_Entry.setDouble(frontRight.getAbsoluteEncoderRadiansNoOffset());
                abs_Enc_FL_Raw_Entry.setDouble(frontLeft.getAbsoluteEncoderRadiansNoOffset());
                abs_Enc_BR_Raw_Entry.setDouble(backRight.getAbsoluteEncoderRadiansNoOffset());
                abs_Enc_BL_Raw_Entry.setDouble(backLeft.getAbsoluteEncoderRadiansNoOffset());

                enc_FR_pos_Entry.setDouble(frontRight.getTurnPosition());
                enc_FL_pos_Entry.setDouble(frontLeft.getTurnPosition());
                enc_BR_pos_Entry.setDouble(backRight.getTurnPosition());
                enc_BL_pos_Entry.setDouble(backLeft.getTurnPosition());
        }

        public void printWorld() {
                System.out.println("Hello World!");
        }

        public double getPitch() {
                return gyro.getPitch() - 1.14;
        }

        /**
         * Used only in characterizing. Don't touch this.
         * 
         * @param direction
         * @return the quasistatic characterization test
         */
        // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        //         return sysIdRoutine.quasistatic(direction);
        // }

        /**
         * Used only in characterizing. Don't touch this.
         * 
         * @param direction
         * @return the dynamic characterization test
         */
        // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        //         return sysIdRoutine.dynamic(direction);
        // }

        /**
         * Used only in Characterizing. Don't touch this. Sets the provided voltages and
         * locks the wheels to 0 radians.
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

        public void printPos2d() {
                System.out.println(poseEstimator.getEstimatedPosition());
        }

        public double getRoll() {
                return gyro.getRoll();
        }

        public Command halt() {
                return Commands.runOnce(() -> {
                }, this);
        }

        public void setspeeds(double speed) {
                frontRight.setDriveVelocity(speed);
                frontLeft.setDriveVelocity(speed);
                backRight.setDriveVelocity(speed);
                backLeft.setDriveVelocity(speed);
        }

public void setInitialAngle(){

}


}

// // Mutable holder for unit-safe voltage values, persisted to avoid
// reallocation.
// private final MutableMeasure<Voltage> m_appliedVoltage =
// mutable(Volts.of(0));
// // Mutable holder for unit-safe linear distance values, persisted to avoid
// // reallocation.
// private final MutableMeasure<Angle> m_distance = mutable(Radians.of(0));
// // Mutable holder for unit-safe linear velocity values, persisted to avoid
// // reallocation.
// private final MutableMeasure<Velocity<Angle>> m_velocity =
// mutable(RadiansPerSecond.of(0));

// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
// private final MutableMeasure<Voltage> m_appliedVoltage =
// mutable(Volts.of(0));
// // Mutable holder for unit-safe linear distance values, persisted to avoid
// // reallocation.
// private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
// // Mutable holder for unit-safe linear velocity values, persisted to avoid
// // reallocation.
// private final MutableMeasure<Velocity<Distance>> m_velocity =
// mutable(MetersPerSecond.of(0));

// private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
// // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
// new SysIdRoutine.Config(),
// new SysIdRoutine.Mechanism(
// // Tell SysId how to plumb the driving voltage to the motors.
// (Measure<Voltage> volts) -> {
// setDriveVoltages(volts);
// },
// // Tell SysId how to record a frame of data for each motor on the mechanism
// // being
// // characterized.
// log -> {
// // Record a frame for the left motors. Since these share an encoder, we
// // consider
// // the entire group to be one motor.
// log.motor("drive-motors")
// .voltage(
// m_appliedVoltage.mut_replace(
// Volts.of(frontRight
// .getTurnVoltage())))
// .linearPosition(
// m_distance.mut_replace(
// frontRight.getTurnEncoderDistance(),
// Meters))
// .linearVelocity(
// m_velocity.mut_replace(
// frontRight.getTurnEncoderVelocity(),
// MetersPerSecond));
// },
// // Tell SysId to make generated commands require this subsystem, suffix test
// // state in
// // WPILog with this subsystem's name ("drive")
// this));

// Autonomous
// Odometer used to get Pose2d of the robot.

// xPID and yPID should have the same values.

// PPHolonomicDriveController holonomicDriveController = new
// PPHolonomicDriveController(xPID, yPID, turnPID);

/**
 * Follows a PathPlanner path. Referenced in autonomous classes.
 *
 * @param traj      The PathPlannerTrajectory to be followed.
 * @param firstPath Whether or not this is the first path being followed in
 *                  auto. If so, resets the gyro before starting.
 * @return A command that follows a path.
 */

// docs:
// https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage

// * I copied this one from documentation */
// public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
// isFirstPath) {
// return new SequentialCommandGroup(
// new InstantCommand(() -> {
// // Reset odometry for the first path you run during auto
// if (isFirstPath) {
// this.setOdometry(traj.getInitialHolonomicPose());
// }
// }),
// new WaitCommand(1),
// new FollowPathWithEvents(new PPSwerveControllerCommand(
// traj,
// this::getPose, // Pose supplier
// RobotMap.DRIVE_KINEMATICS, // SwerveDriveKinematics
// xPID, // X controller. Tune these values for your robot. Leaving them 0
// // will only use feedforwards.
// yPID, // Y controller (usually the same values as X controller)
// turnPID, // Rotation controller. Tune these values for your robot.
// // Leaving them 0 will only use feedforwards.
// this::setModuleStates, // Module states consumer
// true, // Should the path be automatically mirrored depending on alliance
// // color. Optional, defaults to true
// this // Requires this drive subsystem
// ), traj.getMarkers(),
// RobotContainer.eventMap));
// }

// public Command moveCommand() {
// List<Translation2d> midpts = new ArrayList<Translation2d>();
// midpts.add(new Translation2d(1, 0));
// TrajectoryConfig trajectoryConfig = new
// TrajectoryConfig(RobotMap.MAX_DRIVE_SPEED_METERS_PER_SECOND, 1)
// .setKinematics(RobotMap.DRIVE_KINEMATICS);
// Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,
// 0, new Rotation2d(0)),
// midpts, new Pose2d(2.5, 0, new Rotation2d(0)), trajectoryConfig);
// return new SwerveControllerCommand(trajectory, this::getPose,
// RobotMap.DRIVE_KINEMATICS, xPID, yPID,
// turnPIDProfiled, this::setModuleStates, this);
// }
