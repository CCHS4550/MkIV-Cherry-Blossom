// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;

public class SwerveModule extends SubsystemBase {

    private CCSparkMax driveMotor;
    private CCSparkMax turningMotor;

    private PIDController turningPIDController, drivingPIDController;

    private AnalogEncoder absoluteEncoder;
    private double absoluteEncoderOffset;


  /** Creates a new SwerveModule. */
  public SwerveModule
  (String name, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset)
  {
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

    driveMotor = new CCSparkMax(name + "_driveMotor", name + "dM", driveMotorId, MotorType.kBrushless, IdleMode.kCoast, driveMotorReversed);
    turningMotor = new CCSparkMax(name + "_turningMotor", name + "tM", turningMotorId, MotorType.kBrushless, IdleMode.kCoast, turningMotorReversed);
    

    // "Proportional term alone does a good job, so no need for kI or jD"
    drivingPIDController = new PIDController(1, 0, 0);
    turningPIDController = new PIDController(0.5, 0, 0);
    turningPIDController.enableContinuousInput(0, 2 * Math.PI);

    // Rotations -> Meters
    driveMotor.setPositionConversionFactor(RobotMap.WHEEL_ROTATIONS_TO_DRIVE_MOTOR_ROTATIONS * RobotMap.WHEEL_CIRCUMFRENCE);
    // Rotations -> Radians
    turningMotor.setPositionConversionFactor(RobotMap.WHEEL_ROTATIONS_TO_TURN_MOTOR_ROTATIONS * (2 * Math.PI) % (2 * Math.PI));

    // RPM -> Meters per Second
    driveMotor.setVelocityConversionFactor((RobotMap.WHEEL_ROTATIONS_TO_DRIVE_MOTOR_ROTATIONS * RobotMap.WHEEL_CIRCUMFRENCE) / 60);
    // RPM -> Radians per Second
    turningMotor.setVelocityConversionFactor((RobotMap.WHEEL_ROTATIONS_TO_TURN_MOTOR_ROTATIONS * (2 * Math.PI) % (2 * Math.PI)) / 60);
  }

  public double getDrivePosition() {
    return driveMotor.getPosition(); // In meters
  }

  public double getTurningPosition() {
    return turningMotor.getPosition(); // In radians
  }

  public double getDriveVelocity() {
    return driveMotor.getVelocity(); // In meters per second
  }
  public double getTurningVelocity() {
    return turningMotor.getVelocity(); // In radians
  }

  //Absolute Encoder Position in Radians WITH the Absolute Encoder Offset
  public double getAbsoluteEncoderRadiansOffset() { // In Radians
    return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition()) - absoluteEncoderOffset;
  }
  
  //Absolute Encoder Position in Radians WITHOUT the Absolute Encoder Offset
  public double getAbsoluteEncoderRadians() {  // In Radians
    return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition());
  }


  public void resetEncoders() {
    driveMotor.reset();
    turningMotor.setPosition(getAbsoluteEncoderRadiansOffset());
  }
  
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void SetDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / RobotMap.MAX_DRIVE_SPEED_METERS_PER_SECOND);
    turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
