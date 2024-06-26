// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class PneumaticsSystem extends SubsystemBase {
  private boolean airCompressorStatus;
  private DoubleSolenoid.Value pressureSealStatus;

  private CCSparkMax airCompressors = new CCSparkMax("aircompressors", "ac", 5, MotorType.kBrushed, IdleMode.kBrake, false);
  private Compressor compressorFan = new Compressor(Constants.PneumaticsConstants.COMPRESSOR_FAN, PneumaticsModuleType.REVPH);

  private DoubleSolenoid pressureSeal = new DoubleSolenoid
  (
    PneumaticsModuleType.REVPH,
    Constants.PneumaticsConstants.PRESSURE_SEAL[0],
    Constants.PneumaticsConstants.PRESSURE_SEAL[1]
  );
  private Solenoid solenoidValve = new Solenoid
  (
    PneumaticsModuleType.REVPH,
    Constants.PneumaticsConstants.SOLENOID_VALVE
  );

  /** Creates a new Pneumatics. */
  public PneumaticsSystem() {
    airCompressorStatus = false;
    pressureSealStatus = pressureSeal.get();
  }

  // Takes a percentage of 1.0 and sets the air compressors to that percentage along with turning on the accompanying fan if the value is greater than 0. 
  private void setPercentage(double percentage) {

    airCompressors.setVoltage(percentage * 12);

    if (percentage > 0) 
    {
      compressorFan.enableDigital();
    } 
    else 
    {
      compressorFan.disable();
    }
  }
  
// returns a Command to turn on/off both the air compressors and the fan.
  public Command toggleAirCompressors(){
    
    if (airCompressorStatus)  
    {
      airCompressorStatus = !airCompressorStatus;
      return runOnce(() -> setPercentage(0));
    } 
    else 
    {  
      airCompressorStatus = !airCompressorStatus;
      return runOnce(() -> setPercentage(1.0));
    }
    
  }

  public Command disablePressureSeal(){
    return runOnce(() -> pressureSeal.set(Value.kReverse));
  }

  public Command enablePressureSeal(){
    return runOnce(() -> pressureSeal.set(Value.kForward));
  }

  public Command togglePressureSeal(){
    return runOnce(() -> pressureSeal.toggle());

  }

  public Command shoot(){
    return runEnd(() -> solenoidValve.set(true),() -> solenoidValve.set(false)).withTimeout(3);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
