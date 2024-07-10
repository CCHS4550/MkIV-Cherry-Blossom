// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class Declination extends SubsystemBase {

  double declinationSpeedModifier = 0.1;

  double upperBound = 0.0;
  double lowerBound = Math.PI / 2;

  // returns Math.PI/4
  double middlePoint = upperBound + lowerBound / 2;
  // returns Math.PI/2
  double range = Math.abs(upperBound - lowerBound);

  // Should return distance from edge.
  // Ex. pitchLocation = PI/2
  //  1 - Abs(PI/2 - PI/4) / PI/4
  //  1 - PI/4 / PI/4 = 0
  // Ex. pitchLocation = 0
  //  1 - Abs(0-PI/4) / PI/4
  //  1 - PI/4 / PI/4 = 0
  // double distanceFromEdge = 1 - (Math.abs(pitchLocation - middlePoint) / (range / 2));
  // double maxOutwardSpeed = distanceFromEdge / 1;

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
          // 25:1 Gearbox
          // 200:10 Rack
          (2 * Math.PI) / 500,
          ((2 * Math.PI) / 500) / 60);

  private CCSparkMax declination2 =
      new CCSparkMax(
          "pitchMotor2",
          "pM2",
          Constants.MotorConstants.DECLINATION[1],
          MotorType.kBrushless,
          IdleMode.kCoast,
          true,
          1 / 500,
          (1 / 500) / 60);

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
  public Declination() {

    declination1.setPosition(0);
    declination2.setPosition(0);
  }

  public void declinationDefaultMethod(CommandXboxController controller) {
    double declinationSpeed =
        MathUtil.applyDeadband(-controller.getRightY(), 0.05) * declinationSpeedModifier;

    System.out.println(pitchLimitSwitch.get());
    // if (pitchLimitSwitch.get() && pitchLocation < (Math.PI/2)) {
    declination2.set(
        declinationSpeed
        // this.normalizeSpeed(declinationSpeed, pitchLocation)
        );
    // }

    this.checkZeroPitch();
  }

  private void checkZeroPitch() {

    if (!pitchLimitSwitch.get()) {

      declination1.setPosition(0);

      // CCSparkMax.java makes it so that .getPosition() returns a value in Radians

    }
  }

  public double getDeclination() {
    return declination1.getPosition();
  }

  private boolean pastMidpoint(double location) {
    if (location > middlePoint) {
      return true;
    } else {
      return false;
    }
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
    System.out.println("1:" + declination1.getPosition());
    System.out.println("2:" + declination2.getPosition());
  }
}
