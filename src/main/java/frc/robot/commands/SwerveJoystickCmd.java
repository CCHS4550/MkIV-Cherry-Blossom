// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class SwerveJoystickCmd extends Command {

  private final SwerveDrive swerveDrive;
  private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
  private static boolean orientationLocked;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final CommandXboxController driverJoystick;

  private static DoubleSupplier driveSpeedModifier = () -> 0.75;

  private static double turnSpeedModifier = 0.75;


  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd
  (SwerveDrive swerveDrive, DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction, CommandXboxController driverJoystick) {
    this.swerveDrive = swerveDrive;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.xLimiter = new SlewRateLimiter(RobotMap.DRIVE_RATE_LIMIT);
    this.yLimiter = new SlewRateLimiter(RobotMap.DRIVE_RATE_LIMIT);
    this.turningLimiter = new SlewRateLimiter(RobotMap.TURN_RATE_LIMIT);
    this.driverJoystick = driverJoystick; 
    orientationLocked = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driverJoystick.x().onTrue(runOnce(() -> toggleOrientationLock(swerveDrive)));


    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.getAsDouble();
    double ySpeed = ySpdFunction.getAsDouble();
    double turningSpeed = turningSpdFunction.getAsDouble();

    // 2. Apply Deadband
    xSpeed = Math.abs(xSpeed) > ControlMap.ZERO ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > ControlMap.ZERO ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > ControlMap.ZERO ? turningSpeed : 0.0;

    // 3. Make driving smoother.
    xSpeed = xLimiter.calculate(xSpeed) * driveSpeedModifier.getAsDouble();
    ySpeed = yLimiter.calculate(ySpeed) * driveSpeedModifier.getAsDouble();
    turningSpeed = turningLimiter.calculate(turningSpeed) * turnSpeedModifier;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (orientationLocked) {
      // Field Oriented
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveDrive.getRotation2d());
    } else {
      // Robot oriented
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to induvidal module states
    SwerveModuleState[] moduleStates = RobotMap.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveDrive.setModuleStates(moduleStates);
  }

  private static void toggleOrientationLock(SwerveDrive swerveDrive) {

    orientationLocked = !orientationLocked;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
