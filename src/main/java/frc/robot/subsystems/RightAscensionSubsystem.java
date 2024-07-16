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
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.commands.defaultcommands.RightAscensionDefault;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class RightAscensionSubsystem extends SubsystemBase {

  AimSimulator aimer;

  double rightAscensionSpeedModifier = .25;
  double turretLocation;
  double turretOffset;

  double leftBound = -10;
  double rightBound = 10;
  double middlePoint = leftBound + rightBound / 2;
  double range = Math.abs(leftBound - middlePoint);

  double kS = 0;
  double kV = 72.552;
  double kA = 41.245;

  private CCSparkMax rightAscensionMotor =
      new CCSparkMax(
          "yawMotor",
          "yM",
          Constants.MotorConstants.RIGHT_ASCENSION,
          MotorType.kBrushless,
          IdleMode.kBrake,
          false,
          ((2 * Math.PI) / 50),
          ((2 * Math.PI) / 50) / 60);

  private DigitalInput hallEffectSensor = new DigitalInput(Constants.SensorMiscConstants.YAW_SENSOR);
  
  private final Unit<Velocity<Voltage>> VoltsPerSecond = Volts.per(Second);

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(.5),
              Volts.of(1),
              Seconds.of(4),
              (state) ->
                  org.littletonrobotics.junction.Logger.recordOutput(
                      "SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setTurretVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  /** Creates a new RightAscension Subsystem. */
  public RightAscensionSubsystem(AimSimulator aimer) {

    this.aimer = aimer;

    turretLocation = 0.0;
    turretOffset = 0.0;
    rightAscensionMotor.setVoltage(6);
    rightAscensionMotor.set(0);
    rightAscensionMotor.setPosition((3 * Math.PI) / 4);

    setDefaultCommand(new RightAscensionDefault(this));
  }

  public void rightAscensionDefaultMethod(AimSimulator aimer) {}

  private boolean atZero() {

    if (!hallEffectSensor.get()) {
      // System.out.println(rightAscensionMotor.getPosition());
      return true;
    }
    return false;
  }

  public double getRightAscension() {
    return rightAscensionMotor.getPosition();
  }

  public void printRightAscension() {
    System.out.println("Right Ascension:" + rightAscensionMotor.getPosition());
  }

  public void zeroEncoders() {
    rightAscensionMotor.setPosition(0);
  }

  public boolean checkBounds(double x) {
    if (Math.abs(x) < Math.PI) {
      return true;
    } else {
      return false;
    }
  }

  public void setTurretVoltage(Measure<Voltage> volts) {
    rightAscensionMotor.setVoltage(volts.magnitude());
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    System.out.println("awhesfkjldg");
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
    System.out.println("RightAscension: " + hallEffectSensor.get());
  }
}
