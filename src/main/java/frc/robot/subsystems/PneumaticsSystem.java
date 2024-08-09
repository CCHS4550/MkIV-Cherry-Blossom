// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
// import frc.robot.subsystems.swervedrive.SwerveDrive;

public class PneumaticsSystem extends SubsystemBase {

  // SparkAnalogSensor transducer = getDriveAnalog();

  private boolean airCompressorStatus;
  public boolean pressureSealStatus;

  private CCSparkMax airCompressors =
      new CCSparkMax("aircompressors", "ac", 5, MotorType.kBrushed, IdleMode.kBrake, false, false);
  private Compressor compressorFan =
      new Compressor(Constants.PneumaticsConstants.COMPRESSOR_FAN, PneumaticsModuleType.REVPH);

  /*
   * pressureSeal.get().toString() Value.kReverse - The pressureSeal is disabled and the indexer can move.
   * pressureSeal.get().toString() Value.kForward - The pressureSeal is enabled and ready to fire.
   */
  private DoubleSolenoid pressureSeal =
      new DoubleSolenoid(
          Constants.PneumaticsConstants.COMPRESSOR_FAN,
          PneumaticsModuleType.REVPH,
          Constants.PneumaticsConstants.PRESSURE_SEAL[0],
          Constants.PneumaticsConstants.PRESSURE_SEAL[1]);
  /*
   * solenoidValve.get() = false - The shooting valve is closed.
   * solenoidValve.get() = true - The shooting valve is open.
   */
  private Solenoid solenoidValve =
      new Solenoid(
          Constants.PneumaticsConstants.COMPRESSOR_FAN,
          PneumaticsModuleType.REVPH,
          Constants.PneumaticsConstants.SOLENOID_VALVE);

  SparkAnalogSensor transducer = airCompressors.getAnalog();

  public int psi;
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  /** Creates a new Pneumatics. */
  public PneumaticsSystem() {
    pressureSeal.set(Value.kReverse);
    solenoidValve.set(false);
    airCompressorStatus = false;
    compressorFan.disable();
    pressureSealStatus = false;
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

  public Command disablePressureSealCommand() {
    return new InstantCommand(() -> pressureSeal.set(Value.kReverse), this);
  }

  public Command enablePressureSealCommand() {
    return new InstantCommand(() -> pressureSeal.set(Value.kForward), this);
  }

  public void disablePressureSeal() {
    pressureSeal.set(Value.kForward);
    System.out.println("Disabled!");
  }

  public void enablePressureSeal() {
    pressureSeal.set(Value.kReverse);
    System.out.println("Enabled!");
  }

  // public Command togglePressureSeal() {
  //   return new InstantCommand(
  //       () -> {
  //         if (!pressureSealStatus) pressureSeal.set(DoubleSolenoid.Value.kForward);
  //         else {
  //           pressureSeal.set(DoubleSolenoid.Value.kReverse);
  //         }
  //         pressureSealStatus = !pressureSealStatus;
  //       },
  //       this);
  // }
  public void togglePressureSeal() {
    pressureSeal.toggle();
  }

  public Command toggleShoot() {
    return new InstantCommand(() -> solenoidValve.toggle());
  }

  private int checkPressure() {

    /* Found by graphing the transducer voltage against the read psi on the pressure gauge. Graph and find the Linear regression. Courtesy of Dr. Harrison's Physics Class */
    psi = (int) ((54 * transducer.getVoltage()) - 12.2);
    psi = (int) filter.calculate(psi);
    return psi;
    // SmartDashboard.putNumber("Transducer Voltage", transducer.getVoltage());

  }

  @Override
  public void periodic() {
    // System.out.println(transducer.getVoltage());
    checkPressure();
    SmartDashboard.putNumber("Tank Pressure", psi);
    SmartDashboard.putString("pressureSeal Status", pressureSeal.get().toString());
    SmartDashboard.putBoolean("shootingSolenoid Status", solenoidValve.get());

    // SmartDashboard.putBoolean("solenoid", solenoidValve.get());
    // System.out.println(pressureSeal.get());
    // System.out.println(solenoidValve.get());
    // This method will be called once per scheduler run
  }
}
