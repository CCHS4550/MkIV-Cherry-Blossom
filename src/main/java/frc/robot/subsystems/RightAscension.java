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

  double rightAscensionSpeedModifier = .25;
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
    false,
    ((2 * Math.PI) / 50),
    ((2 * Math.PI) / 50) / 60 
  );
  private DigitalInput hallEffectSensor = new DigitalInput(1);
  


  /** Creates a new RightAscension. */
  public RightAscension() {
    turretLocation = 0.0;
    turretOffset = 0.0;
    rightAscensionMotor.setVoltage(6);
    rightAscensionMotor.set(0.05);
    rightAscensionMotor.setPosition((3*Math.PI)/4);
    
  }

  public void rightAscensionDefaultMethod(CommandXboxController controller, AimSimulator aimer) {
    
    double controllerInput = MathUtil.applyDeadband(controller.getRightX(), 0.05);
    

  }

  public void rightAscensionDefaultMethodOutDated(CommandXboxController controller){

    // System.out.println("controller input" + controller.getRightX());
    double rightAscensionSpeed = MathUtil.applyDeadband(controller.getRightX(), 0.05) * rightAscensionSpeedModifier;

    // System.out.println("rightascensionspeed:" + rightAscensionSpeed);
    if (rightAscensionMotor.getPosition() < ((3*Math.PI)/2) && rightAscensionMotor.getPosition() > 0) {

      rightAscensionMotor.set(rightAscensionSpeed);

    } else if(rightAscensionMotor.getPosition() > (3*Math.PI/2)) {

      rightAscensionMotor.set(-.25);

    } else {

      rightAscensionMotor.set(.25);
    }

    // System.out.println("speed" + rightAscensionMotor.get());
    // System.out.println("position" + rightAscensionMotor.getPosition());


    // this.checkZeroYaw();

    // in Radians
    turretLocation = Math.abs(rightAscensionMotor.getPosition()) - turretOffset;

  }

  private void checkZeroYaw() {
    // returns value 0-1

    if (hallEffectSensor.get()) {
      // System.out.println(rightAscensionMotor.getPosition());
      rightAscensionMotor.setPosition((3*Math.PI)/4);
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

  public boolean checkBounds(double x){
    if (x < ((Math.PI * 3) / 2) && x > 0) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // printEncoders();
  }

}
