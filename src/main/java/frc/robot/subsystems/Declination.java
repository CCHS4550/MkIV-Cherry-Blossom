// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class Declination extends SubsystemBase {

  DoubleSupplier declinationSpeedModifier = () -> 0.75;
  Double pitchLocation;
  Double pitchOffset1;
  Double pitchOffset2;



  private CCSparkMax declination1 = new CCSparkMax
  (
  "pitchMotor1",
  "pM1",
  Constants.MotorConstants.DECLINATION[0],
  MotorType.kBrushless,
  IdleMode.kCoast,
  false
  );
  private CCSparkMax declination2 = new CCSparkMax
  (
  "pitchMotor2",
  "pM2",
  Constants.MotorConstants.DECLINATION[1],
  MotorType.kBrushless,
  IdleMode.kCoast,
  true
  );

  private DigitalInput pitchLimitSwitch = new DigitalInput(9);
  PIDController pitchPidController;


  /** Creates a new Declination. */
  public Declination() {
    pitchPidController = new PIDController(1.2, 0, 0);
    pitchLocation = 0.0;
    pitchOffset1 = 0.0;
    pitchOffset2 = 0.0;
  }

  public void declinationDefaultMethod(CommandXboxController controller) {
    double declinationSpeed = MathUtil.applyDeadband(-controller.getRightY(), 0.15) * declinationSpeedModifier.getAsDouble();

    if (pitchLocation < Math.PI/2) {
      declination1.set
      (
        declinationSpeed * pitchPidController.calculate(pitchLocation, Math.PI/2)
      );
    }

    this.checkZeroPitch();

    pitchLocation = Math.abs(((declination1.getPosition() / 126) % 1) * (2 * Math.PI)) - pitchOffset1;

  }

  private void checkZeroPitch(){
    
    if (pitchLimitSwitch.get()) {
      // 42 ticks per motor revolution
      // 3 motor revolutions per pitch revolution (Don't push it past a quarter of its revolutions!)
      // 126 ticks per pith revolution
      // In radians
      pitchOffset1 = Math.abs(((declination1.getPosition() / 126) % 1) * (2 * Math.PI));
      pitchOffset2 = Math.abs(((declination2.getPosition() / 126) % 1) * (2 * Math.PI));
      
      
    }
  }

  public double getDeclination(){
    return pitchLocation;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
