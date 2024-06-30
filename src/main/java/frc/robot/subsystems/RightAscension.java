// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class RightAscension extends SubsystemBase {

  double rightAscensionSpeedModifier = 1;
  double turretLocation;
  double turretOffset;
  
  double leftBound = -10;
  double rightBound = 10;
  double middlePoint = leftBound + rightBound / 2;
  double range = Math.abs(leftBound - middlePoint);


  private CCSparkMax rightAscensionMotor = new CCSparkMax
  (
    "yawMotor",
    "yM",
    Constants.MotorConstants.RIGHT_ASCENSION,
    MotorType.kBrushless,
    IdleMode.kBrake,
    false
  );
  private DigitalInput hallEffectSensor = new DigitalInput(1);
  


  /** Creates a new RightAscension. */
  public RightAscension() {
    turretLocation = 0.0;
    turretOffset = 0.0;
    rightAscensionMotor.setVoltage(6);
    rightAscensionMotor.set(0.1);
    rightAscensionMotor.setPosition((3*Math.PI)/4);
    
  }


  public void rightAscensionDefaultMethod(CommandXboxController controller){


    
    double rightAscensionSpeed = MathUtil.applyDeadband(-controller.getRightX(), 0.07) * rightAscensionSpeedModifier;

    if (rightAscensionMotor.getPosition() < ((3*Math.PI)/2) && rightAscensionMotor.getPosition() > 0) {
      rightAscensionMotor.set(rightAscensionSpeed);
    }


    this.checkZeroYaw();

    // in Radians
    turretLocation = Math.abs(rightAscensionMotor.getPosition()) - turretOffset;

  }

  private void checkZeroYaw() {
    // returns value 0-1
    // double hallEffectInput = MathUtil.applyDeadband(hallEffectSensor.get(), 0.1);

    if (hallEffectSensor.get()) {
      System.out.println(rightAscensionMotor.getPosition());
      rightAscensionMotor.setPosition((3*Math.PI)/4);
    // In radians
      // turretOffset = Math.abs(((rightAscensionMotor.getPosition())) * (2 * Math.PI));
    }

  }
    
  

  public double getRightAscension() {
    return turretLocation;
  }

  public void printEncoders() {
    System.out.println("Right Ascension:" + rightAscensionMotor.getPosition());
  }

  public void zeroEncoders() {
    rightAscensionMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printEncoders();
  }

}
