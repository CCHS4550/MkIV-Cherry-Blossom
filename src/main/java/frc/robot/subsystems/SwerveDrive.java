// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.maps.RobotMap;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule
  ("frontLeft",
   RobotMap.FRONT_LEFT_DRIVE, 
   RobotMap.FRONT_LEFT_TURN,
    RobotMap.FRONT_LEFT_DRIVE_REVERSE,
     RobotMap.FRONT_LEFT_TURN_REVERSE,
      RobotMap.FRONT_LEFT_ABSOLUTE_ENCODER,
       RobotMap.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET);

  private final SwerveModule frontRight = new SwerveModule
  ("frontRight",
   RobotMap.FRONT_RIGHT_DRIVE, 
   RobotMap.FRONT_RIGHT_TURN,
    RobotMap.FRONT_RIGHT_DRIVE_REVERSE,
     RobotMap.FRONT_RIGHT_TURN_REVERSE,
      RobotMap.FRONT_RIGHT_ABSOLUTE_ENCODER,
       RobotMap.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET);

  private final SwerveModule backLeft = new SwerveModule
  ("backLeft",
   RobotMap.BACK_LEFT_DRIVE, 
   RobotMap.BACK_LEFT_TURN,
    RobotMap.BACK_LEFT_DRIVE_REVERSE,
     RobotMap.BACK_LEFT_TURN_REVERSE,
      RobotMap.BACK_LEFT_ABSOLUTE_ENCODER,
       RobotMap.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET);

  private final SwerveModule backRight = new SwerveModule
  ("backRight",
   RobotMap.BACK_RIGHT_DRIVE, 
   RobotMap.BACK_RIGHT_TURN,
    RobotMap.BACK_RIGHT_DRIVE_REVERSE,
     RobotMap.BACK_RIGHT_TURN_REVERSE,
      RobotMap.BACK_RIGHT_ABSOLUTE_ENCODER,
       RobotMap.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);


  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Resets the NavX as the robot resets! 
    // Pauses to let the gyro recalibrate before resetting.
    // Within a thread so as to not disturb other processes from running.
    new Thread( () -> {
    try { 
      Thread.sleep(1000);
      zeroHeading();
    } catch (Exception e) {}
    }).start();

    }




  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    // Similar to modulus %
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }


  // Takes an array of SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotMap.MAX_DRIVE_SPEED_METERS_PER_SECOND);
    frontLeft.SetDesiredState(desiredStates[0]);
    frontRight.SetDesiredState(desiredStates[1]);
    backLeft.SetDesiredState(desiredStates[2]);
    backRight.SetDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
