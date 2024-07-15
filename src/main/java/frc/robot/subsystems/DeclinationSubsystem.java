// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.commands.defaultcommands.DeclinationDefault;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class Declination extends SubsystemBase {

  AimSimulator aimer;
  double declinationSpeedModifier = 0.1;

  private DigitalInput pitchLimitSwitch =
      new DigitalInput(Constants.SensorMiscConstants.PITCH_LIMIT_SWITCH);

  private CCSparkMax declination1 =
      new CCSparkMax(
          "pitchMotor1",
          "pM1",
          Constants.MotorConstants.DECLINATION[0],
          MotorType.kBrushless,
          IdleMode.kCoast,
          false,
          // 4:1 Gearbox
          // 200:10 Rack
          // For some reason, only works in decimal form!
          (2 * Math.PI) * 0.0125,
          ((2 * Math.PI) * 0.0125 * 0.0166));

  private CCSparkMax declination2 =
      new CCSparkMax(
          "pitchMotor2",
          "pM2",
          Constants.MotorConstants.DECLINATION[1],
          MotorType.kBrushless,
          IdleMode.kCoast,
          true,
          ((Math.PI * 2) * 0.0125),
          ((Math.PI * 2) * 0.0125 * 0.0166));

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(1),
              Volts.of(5),
              Seconds.of(4),
              (state) ->
                  org.littletonrobotics.junction.Logger.recordOutput(
                      "SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setPitchVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  /** Creates a new Declination. */
  public Declination(AimSimulator aimer) {

    this.aimer = aimer;

    declination1.setPosition(0);
    declination2.setPosition(0);

    setDefaultCommand(new DeclinationDefault(this));
  }

  public void declinationDefaultMethod(boolean isUp) {

    int direction = -1;
    if (isUp) {
      direction = 1;
    }

    if (!limitSwitchPressed()) {

      declination1.set(direction * declinationSpeedModifier);
      declination2.set(direction * declinationSpeedModifier);
    }

    // System.out.println(pitchLimitSwitch.get());

  }

  private boolean limitSwitchPressed() {

    if (!pitchLimitSwitch.get()) {

      return true;

      // CCSparkMax.java makes it so that .getPosition() returns a value in Radians

    } else {

      return false;
    }
  }

  public double getDeclination() {
    return declination1.getPosition();
  }

  // private double normalizeSpeed(double speed, double location) {
  //   if (speed > 0 && this.pastMidpoint(location)) {
  //     return OI.normalize(speed, 0, );
  //   } else if (speed < 0 && !this.pastMidpoint(locatiomaxOutwardSpeedn)) {
  //     return OI.normalize(speed, -maxOutwardSpeed, 0);
  //   } else {
  //     return speed;
  //   }
  // }

  public void printEncoders() {
    System.out.println("Declination:" + declination1.getPosition());
  }

  public void setPitchVoltage(Measure<Voltage> volts) {
    declination1.setVoltage(volts.magnitude());
    declination2.setVoltage(volts.magnitude());
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the dynamic characterization test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // printEncoders();
    // System.out.println("1:" + declination1.getPosition());
    // System.out.println("2:" + declination2.getPosition());
    System.out.println(pitchLimitSwitch.get());
  }
}
