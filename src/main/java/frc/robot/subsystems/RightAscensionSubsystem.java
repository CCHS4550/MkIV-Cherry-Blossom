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
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.maps.Constants;

public class RightAscensionSubsystem extends SubsystemBase {

  public static RightAscensionSubsystem mInstance;

  public static RightAscensionSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new RightAscensionSubsystem();
    }
    return mInstance;
  }

  private final Unit<Velocity<Voltage>> VoltsPerSecond = Volts.per(Second);

  AimSimulator aimer;

  double rightAscensionSpeedModifier = .25;
  double turretLocation;
  double turretOffset;

  SimpleMotorFeedforward rightAscensionFeedForward =
      new SimpleMotorFeedforward(
          Constants.FeedForwardConstants.RIGHT_ASCENSION_KS,
          Constants.FeedForwardConstants.RIGHT_ASCENSION_KV,
          Constants.FeedForwardConstants.RIGHT_ASCENSION_KA);

  PIDController rightAscensionFeedback = new PIDController(9, 1.5, 1.25);

  private TrapezoidProfile.Constraints constraints;

  // Creates a Trapezoid Profile.
  // Instead of using one singular setpoint, the Trapezoid Profile creates setpoints to segment the
  // movement.
  private TrapezoidProfile profile;

  // States of the mechanism with position and velocity.
  // Ex. "goal" is our desired final position at a velocity of 0 (See setGoal method!)
  private TrapezoidProfile.State setPoint, goal;

  public CCSparkMax rightAscensionMotor =
      new CCSparkMax(
          "yawMotor",
          "yM",
          Constants.MotorConstants.RIGHT_ASCENSION,
          MotorType.kBrushless,
          IdleMode.kBrake,
          false,
          ((2 * Math.PI) / 50),
          ((2 * Math.PI) / 50) / 60);
  // 1,
  // 1);

  private DigitalInput hallEffectSensor =
      new DigitalInput(Constants.SensorMiscConstants.YAW_SENSOR);

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(1),
              Volts.of(1),
              Seconds.of(2),
              (state) ->
                  org.littletonrobotics.junction.Logger.recordOutput(
                      "SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setTurretVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  /** Creates a new RightAscension Subsystem. */
  private RightAscensionSubsystem() {

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

    // Shuffleboard.getTab("Aimer").add("RightAscension: PID Controller", rightAscensionFeedback);
    SmartDashboard.putData("RightAscension: PID Controller", rightAscensionFeedback);

    // Shuffleboard.getTab("Aimer").add("X Goal", getGoal().position);
    // Shuffleboard.getTab("Aimer").add("X Actual", rightAscensionMotor.getPosition());
  }

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
    // SmartDashboard.putNumber("feedForwardPower", feedForwardPower);

    // The Pid Calculation, calculating a voltage using the current position and the goal position.
    double feedBackPower =
        rightAscensionFeedback.calculate(rightAscensionMotor.getPosition(), targetPosition);
    // SmartDashboard.putNumber("RightAscension: feedBackPower", feedBackPower);

    double totalPower = feedForwardPower + feedBackPower;
    totalPower = OI.normalize(totalPower, -3, 3);
    rightAscensionMotor.setVoltage(totalPower);
    // SmartDashboard.putNumber("RightAscension: totalPower", (totalPower));
    // Sets the current setpoint to the point it will be in the future to prepare for the next time
    // targetPosition() is called.
    setSetpoint(nextSetpoint);
  }

  /**
   * Main Command that utilizes the targetposition method to bring the mechanism to a point.
   *
   * @param goalPosition The ideal final end position
   * @return A runCommand that runs periodically to bring the mechanism to the goalPosition until
   *     within 0.01. Then sets the speed to 0 oncee.
   */
  public Command rightAscensionToPoint(double goalPosition) {
    return this.runEnd(() -> this.targetPosition(goalPosition), () -> setTurretSpeed(0))
        .until(() -> ((Math.abs(goalPosition - rightAscensionMotor.getPosition())) < 0.3));
  }

  // Repeatable version of Main Command
  public void rightAscensionToPointRepeatable(double goalPosition) {
    if (true) {
      // if (!((Math.abs(goalPosition - rightAscensionMotor.getPosition())) < 0.03)) {
      this.targetPosition(goalPosition);
    }
    // if (rightAscensionMotor.getSpeed() > 0) {
    //   setTurretSpeed(0);
    // }
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

  /* Helper Methods for targetPosition(), ___toPoint(), ___toPointRepeatable()
   * Utilizes trapezoid profiles.
   */
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

    SmartDashboard.putNumber("X Goal", getGoal().position);
    SmartDashboard.putNumber("X Actual", rightAscensionMotor.getPosition());

    // rightAscensionToPointRepeatable(aimer.xAngle);
  }
}
