// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
// import frc.robot.subsystems.swervedrive.SwerveDrive;

public class PneumaticsSystem extends SubsystemBase {

  // SparkAnalogSensor transducer = getDriveAnalog();

  private boolean airCompressorStatus;
  private DoubleSolenoid.Value pressureSealStatus;

  private CCSparkMax airCompressors = new CCSparkMax("aircompressors", "ac", 5, MotorType.kBrushed, IdleMode.kBrake,
      false, false);
  private Compressor compressorFan = new Compressor(Constants.PneumaticsConstants.COMPRESSOR_FAN,
      PneumaticsModuleType.REVPH);

  private DoubleSolenoid pressureSeal = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.PneumaticsConstants.PRESSURE_SEAL[0],
      Constants.PneumaticsConstants.PRESSURE_SEAL[1]);
  private Solenoid solenoidValve = new Solenoid(PneumaticsModuleType.REVPH,
      Constants.PneumaticsConstants.SOLENOID_VALVE);

  SparkAnalogSensor transducer = airCompressors.getAnalog();

  // The pressure transducer is a physical component on the robot that returns a
  // voltage, 0-5V, that represents a PSI from 0-150 PSI.
  // The voltage returned is directly proportional to the PSI, so the constants
  // defined below help derive that.
  // "slope" is the ratio of the PSI to the Voltage.
  // ps. Couldn't be bothered to understand the math, but I used this
  // https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
  private double slope = ((150 - 0) / (5 - 0));
  private double input;
  private double psi;

  /** Creates a new Pneumatics. */
  public PneumaticsSystem() {
    airCompressorStatus = false;
    compressorFan.disable();
    pressureSealStatus = pressureSeal.get();
  }

  // Takes a percentage of 1.0 and sets the air compressors to that percentage
  // along with turning on
  // the accompanying fan if the value is greater than 0.
  private void setPercentage(double percentage) {

    airCompressors.setVoltage(percentage * 12);

    if (percentage > 0) {
      compressorFan.enableDigital();
    } else {
      compressorFan.disable();
    }
  }

  // returns a Command to turn on/off both the air compressors and the fan.
  public void toggleAirCompressors() {

    if (airCompressorStatus) {
      airCompressorStatus = !airCompressorStatus;
      this.setPercentage(0);
    } else {
      airCompressorStatus = !airCompressorStatus;
      System.out.println(airCompressorStatus);
      this.setPercentage(1.0);
    }
  }

  public Command disablePressureSeal() {
    return new InstantCommand(() -> pressureSeal.set(Value.kReverse));
  }

  public Command enablePressureSeal() {
    return new InstantCommand(() -> pressureSeal.set(Value.kForward));
  }

  public Command togglePressureSeal() {
    return new InstantCommand(() -> pressureSeal.toggle());
  }

  public Command shoot() {
    System.out.println("Test");
    return new SequentialCommandGroup(
        new InstantCommand(() -> solenoidValve.set(true)),
        new InstantCommand(() -> solenoidValve.set(false)));
  }

  private void checkPressure() {
    input = transducer.getVoltage();
    psi = (0 + slope * (input - 0));
    SmartDashboard.putNumber("Tank Pressure", psi);
  }

  @Override
  public void periodic() {
    System.out.println(transducer.getVoltage());
    checkPressure();
    // This method will be called once per scheduler run
  }
}
