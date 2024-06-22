package frc.maps;

// import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import frc.robot.CCSparkMax;

/*
    RobotMap holds all the ports involved in the robot.
    This ranges from talon ports, all the way to the ports
    on the controllers. This also contains the polarity for the wheels
    and the various ports assoiated with sensors
    If you wish to create your own port, here is the syntax:
        public static final returnType name = value;
    Notes on creating ports:
        1. Ports must be integers or booleans
        2. they MUST be public static final;
        3. If the port is not plugged in, make int values -1, and boolean values false
*/
public interface RobotMap {
        // Swerve Module Constants

        // 1, 2, 3, 4, 5, 6, 7, ,8

        // 4 drive
        // 4 turn - 1

        // in meters
        public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(4 * Math.PI);


        // 150/7 rotations of the turn motor to one turning rotation of the wheel
        // The turn motor completes 150 rotations to every 7 rotations of the Swerve Module
        public static final double WHEEL_ROTATIONS_TO_TURN_MOTOR_ROTATIONS = 7.0 / 150.0 ;

        // How many radians the wheel pivots for one full rotation of the turn motor
        // Conversion of previous variable
        public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS = Units.rotationsToRadians(WHEEL_ROTATIONS_TO_TURN_MOTOR_ROTATIONS);
        public static final double TURN_MOTOR_RADIANS_PER_SECOND = TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS / 60.0;

        // The drive motor will complete 6.75 rotations for every 1 driving rotation of the wheel.
        //Dependent on gear ratio of Swerve Drive Module; For Mk4i L2 it is 6.75:1
        public static final double WHEEL_ROTATIONS_TO_DRIVE_MOTOR_ROTATIONS = 1.0 / 6.75;

        // Horizontal distance travelled by one motor rotation
        // Conversion of previous variable
        public static final double HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION = WHEEL_CIRCUMFRENCE * WHEEL_ROTATIONS_TO_DRIVE_MOTOR_ROTATIONS;
        public static final double DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR = HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION / 60.0;

        // Swerve Drive Base Motor & CAN ID Constants
        public static final int FRONT_RIGHT_DRIVE = 6;
        public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
        public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
        public static final int FRONT_RIGHT_TURN = 5;
        public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
        public static final double FRONT_RIGHT_TURN_ENCODER = 1;

        public static final int FRONT_LEFT_DRIVE = 2;
        public static final boolean FRONT_LEFT_DRIVE_REVERSE = false;
        public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
        public static final int FRONT_LEFT_TURN = 1;
        public static final boolean FRONT_LEFT_TURN_REVERSE = true;
        public static final double FRONT_LEFT_TURN_ENCODER = 1;

        public static final int BACK_RIGHT_DRIVE = 7;
        public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
        public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
        public static final int BACK_RIGHT_TURN = 8;
        public static final boolean BACK_RIGHT_TURN_REVERSE = true;
        public static final double BACK_RIGHT_TURN_ENCODER = 1;

        public static final int BACK_LEFT_DRIVE = 3;
        public static final boolean BACK_LEFT_DRIVE_REVERSE = false;
        public static final double BACK_LEFT_DRIVE_ENCODER = 1;
        public static final int BACK_LEFT_TURN = 4;
        public static final boolean BACK_LEFT_TURN_REVERSE = true;
        public static final double BACK_LEFT_TURN_ENCODER = 1;

    
        // Absolute Encoder Ports
        public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 3;
        public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 0;
        public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 2;
        public static final int BACK_LEFT_ABSOLUTE_ENCODER = 1;


        // Robot Constants (change with SysId)
        // max speed in free sprint: used in getting velocities of swerve modules
        public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL = 4.72;

        // Swerve Module Absolute Encoder Offsets
        // Will have to recalculate for new Swerve Bases 
        public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = (5.968 - Math.PI);
        public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = (1.752 - Math.PI);
        public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = (5.623 - Math.PI);
        public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = (2.378 - Math.PI);

        // Velocity Limits
        public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND = 5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

        // Rate Limiters (acceleration)
        public static final double DRIVE_RATE_LIMIT = MAX_DRIVE_SPEED_METERS_PER_SECOND * 1.5;
        public static final double TURN_RATE_LIMIT = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // public static final PathConstraints AUTO_PATH_CONSTRAINTS = new PathConstraints(
        //                 MAX_DRIVE_SPEED_METERS_PER_SECOND - 2, DRIVE_RATE_LIMIT - 0.3,
        //                 MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        //                 TURN_RATE_LIMIT);
        // public static final PathConstraints AUTO_PATH_CONSTRAINTS = new
        // PathConstraints(4, 3);
        public static final TrapezoidProfile.Constraints thetaControlConstraints = new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TURN_RATE_LIMIT);

        // Robot Dimensions (relative to wheel locations)
        // Since this robot is a square, no need for 2 values. In a non-square chassis,
        // 2 values needed.
        // from drive shaft to drive shaft. 
        // Previous was 24 for 2024 Competition Robot
        public static final double WHEEL_BASE = Units.inchesToMeters(24.75); 


        /** FR FL BR BL. Same as order of swerve module states */
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(WHEEL_BASE / 2, -WHEEL_BASE / 2),
                        new Translation2d(WHEEL_BASE / 2, WHEEL_BASE / 2),
                        new Translation2d(-WHEEL_BASE / 2, -WHEEL_BASE / 2),
                        new Translation2d(-WHEEL_BASE / 2, WHEEL_BASE / 2));

        /* To Do */
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 50;

        // TODO Do sysid to get values
        public static final double DRIVE_KS = 0.14222;
        public static final double DRIVE_KV = 2.5769;
        public static final double DRIVE_KA = 0.29973;

        public static final double TURNKS = 0;
        public static final double TURNKV = 0;

        // * TODO SysId these values */
        public static final double ELEVATOR_KS = 0;
        public static final double ELEVATOR_KG = 0;
        public static final double ELEVATOR_KV = 0;
        public static final double ELEVATOR_KA = 0;

        public static final double WRIST_KS = 0;
        public static final double WRIST_KG = 0;
        public static final double WRIST_KV = 0;
        public static final double WRIST_KA = 0;
        
}