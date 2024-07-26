// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.commands.defaultcommands.RightAscensionDefault;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class RightAscensionSubsystem extends SubsystemBase {

  private final Unit<Velocity<Voltage>> VoltsPerSecond = Volts.per(Second);

  AimSimulator aimer;

  double rightAscensionSpeedModifier = .25;
  double turretLocation;
  double turretOffset;

  double leftBound = -10;
  double rightBound = 10;
  double middlePoint = leftBound + rightBound / 2;
  double range = Math.abs(leftBound - middlePoint);

  double kS = 0;
  double kV = .72552;
  double kA = .41245;

  SimpleMotorFeedforward rightAscensionFeedForward = new SimpleMotorFeedforward(kS, kV);

  PIDController rightAscensionFeedback = new PIDController(0.08, 0, 0);

  private TrapezoidProfile.Constraints constraints;

  // Creates a Trapezoid Profile.
  // Instead of using one singular setpoint, the Trapezoid Profile creates setpoints to segment the
  // movement.
  private TrapezoidProfile profile;

  // States of the mechanism with position and velocity.
  // Ex. "goal" is our desired final position at a velocity of 0 (See setGoal method!)
  private TrapezoidProfile.State setPoint, goal;

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

  private DigitalInput hallEffectSensor =
      new DigitalInput(Constants.SensorMiscConstants.YAW_SENSOR);

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
    rightAscensionMotor.setPosition(0);

    constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
    profile = new TrapezoidProfile(constraints);
    setPoint = new TrapezoidProfile.State();
    goal = new TrapezoidProfile.State();

    setSetpoint(new State(rightAscensionMotor.getPosition(), 0));

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

  // Helper method called repeatedly for rightAscensionToPoint() Method.
  private void targetPosition(double targetPosition) {
    setGoal(targetPosition);

    // First argument = how long to go from current state to the next state.
    // Second argument = mechanism's current state.
    // Third argument = the goal state, defined in setGoal() method.
    // the profile.calculate returns the next setpoint state at the time given (0.02 seconds from
    // now).
    TrapezoidProfile.State nextSetpoint = profile.calculate(0.02, getSetpoint(), getGoal());

    // The Feed Forward Calculation, calculating the voltage for the motor using the position and
    // velocity of the next setpoint.
    double feedForwardPower =
        rightAscensionFeedForward.calculate(nextSetpoint.position, nextSetpoint.velocity);

    // The Pid Calculation, calculating a voltage using the current position and the goal position.
    double feedBackPower =
        rightAscensionFeedback.calculate(rightAscensionMotor.getPosition(), targetPosition);

    rightAscensionMotor.setVoltage(feedForwardPower + feedBackPower);
    SmartDashboard.putNumber("feedForward + feedBack", (feedForwardPower + feedBackPower));
    // Sets the current setpoint to the point it will be in the future to prepare for the next time
    // targetPosition() is called.
    setSetpoint(nextSetpoint);
  }

  // Main command called
  public Command rightAscensionToPoint(double goalPosition) {
    return this.runEnd(() -> this.targetPosition(goalPosition), () -> setTurretSpeed(0))
        .until(() -> ((Math.abs(goalPosition - rightAscensionMotor.getPosition())) < 0.3));
  }

  // Repeatable version of Main Command
  public void rightAscensionToPointRepeatable(double goalPosition) {
    if (!((Math.abs(goalPosition - rightAscensionMotor.getPosition())) < 0.03)) {
      this.targetPosition(goalPosition);
    }
    if (rightAscensionMotor.getSpeed() > 0) {
      setTurretSpeed(0);
    }
  }

  public void setTurretVoltage(Measure<Voltage> volts) {
    rightAscensionMotor.setVoltage(volts.magnitude());
  }

  private void setTurretSpeed(double speed) {
    if (true) {
      rightAscensionMotor.set(speed);
    }
  }

  private void checkHallSensor() {
    if (!hallEffectSensor.get()) {
      zeroEncoder();
    }
  }

  private void zeroEncoder() {
    rightAscensionMotor.setPosition(0);
  }

  public boolean checkBounds(double x) {
    if (Math.abs(x) < Math.PI) {
      return true;
    } else {
      return false;
    }
  }

  public void setSetpoint(TrapezoidProfile.State setPoint) {
    this.setPoint = setPoint;
  }

  public TrapezoidProfile.State getSetpoint() {
    return setPoint;
  }

  public void setGoal(double goalState) {
    goal = new TrapezoidProfile.State(goalState, 0);
  }

  public State getGoal() {
    return goal;
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
    // System.out.println("RightAscension: " + hallEffectSensor.get());
    this.checkHallSensor();
    SmartDashboard.putNumber("X Setpoint", getSetpoint().position);
    SmartDashboard.putNumber("X Actual", rightAscensionMotor.getPosition());
    rightAscensionToPointRepeatable(aimer.xAngle);
  }
}
