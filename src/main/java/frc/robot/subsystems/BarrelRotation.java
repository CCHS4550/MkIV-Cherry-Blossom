// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;

public class BarrelRotation extends SubsystemBase {
  DoubleSupplier barrelRotationSpeedModifier = () -> 0.5;
  Double barrelAngle;

  CCSparkMax barrelRotationMotor = new CCSparkMax("barrelRotationMotor","bRM", 13, MotorType.kBrushless, IdleMode.kBrake, false);

  private PWM hallEffectSensor = new PWM(0);

  /** Creates a new BarrelRotation. */
  public BarrelRotation() {}





  public Command reload() {
        
    return this.runEnd(
      () -> {
        spinBarrels();
      },
      () -> {

      }
    );
  } 

  private void spinBarrels(){
    barrelRotationMotor.set(1 * barrelRotationSpeedModifier.getAsDouble());
  }



  private void checkZero(){
    // returns value 0-1
    double hallEffectInput = MathUtil.applyDeadband(hallEffectSensor.getPosition(), 0.1);
  }

  private void rotateUntilReady(){
    // TODO: need to find how many rotations of motor corresponds to full barrel rotation
    

  }
  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
