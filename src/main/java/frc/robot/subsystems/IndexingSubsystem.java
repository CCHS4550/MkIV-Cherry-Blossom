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
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import java.util.function.DoubleSupplier;

public class IndexingSubsystem extends SubsystemBase {
  PneumaticsSystem pneumatics;
  DoubleSupplier barrelRotationSpeedModifier = () -> 1;
  Double barrelAngle;

  SimpleMotorFeedforward indexFeedForward =
      new SimpleMotorFeedforward(
          Constants.FeedForwardConstants.INDEX_KS,
          Constants.FeedForwardConstants.INDEX_KV,
          Constants.FeedForwardConstants.INDEX_KA);

  /*
   * This is the controller that actually brings the mechanism to the point.
   * Very important to test manually! Google PID tuning to find out how to tune PID constants.
   */
  PIDController indexFeedBack = new PIDController(.9, 0.2, 0.00);

  private TrapezoidProfile.Constraints constraints;

  // Creates a Trapezoid Profile.

  // Instead of using one singular setpoint, the Trapezoid Profile creates
  // setpoints to segment the
  // movement.
  private TrapezoidProfile profile;

  // States of the mechanism with position and velocity.
  // Ex. "goal" is our desired final position at a velocity of 0 (See setGoal
  // method!)
  private TrapezoidProfile.State setPoint, goal;

  private DigitalInput hallEffectSensor =
      new DigitalInput(Constants.SensorMiscConstants.BARREL_SENSOR);

  CCSparkMax indexMotor =
      new CCSparkMax(
          "barrelRotationMotor", "bRM", 13, MotorType.kBrushless, IdleMode.kBrake, false);

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5),
              Volts.of(0.5),
              Seconds.of(1.25),
              (state) ->
                  org.littletonrobotics.junction.Logger.recordOutput(
                      "SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setIndexVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  // Will not work because PWM is only an output!
  // private PWM hallEffectSensor = new PWM(0);

  /** Creates a new IndexingSubsystem. */
  public IndexingSubsystem(PneumaticsSystem pneumatics) {
    this.pneumatics = pneumatics;

    constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
    profile = new TrapezoidProfile(constraints);
    setPoint = new TrapezoidProfile.State();
    goal = new TrapezoidProfile.State();

    setSetpoint(new State(0, 0));
  }

  /**
   * Helper method called repeatedly for declinationToPoint() Method.
   *
   * @param targetPosition the end goal position.
   */
  private void targetPosition(double targetPosition) {
    setGoal(targetPosition);

    /* First argument = how long to go from current state to the next state.
    Second argument = mechanism's current state.
    Third argument = the goal state, defined in setGoal() method.
    the profile.calculate returns the next setpoint state at the time given (0.02 seconds from now).
    */

    TrapezoidProfile.State nextSetpoint = profile.calculate(0.02, getSetpoint(), getGoal());

    // The Feed Forward Calculation, calculating the voltage for the motor using the
    // position and
    // velocity of the next setpoint.
    // Feedforward doesn't seem necessarily necessary?
    double feedForwardPower = 0;
    // declinationFeedForward.calculate(nextSetpoint.position, nextSetpoint.velocity);

    // The Pid Calculation, calculating a voltage using the current position and the
    // goal position.
    double feedBackPower = indexFeedBack.calculate(indexMotor.getPosition(), targetPosition);

    indexMotor.setVoltage(feedForwardPower + feedBackPower);
    SmartDashboard.putNumber("feedForward + feedBack", (feedForwardPower + feedBackPower));
    // Sets the current setpoint to the point it will be in the future to prepare
    // for the next time
    // targetPosition() is called.
    setSetpoint(nextSetpoint);
  }

  // Main command called
  public Command indexToPoint(double goalPosition) {
    return this.runEnd(() -> this.targetPosition(goalPosition), () -> setIndexSpeed(0));
    // .until(() -> ((Math.abs(goalPosition - declination1.getPosition())) < 0.01));
  }

  // Repeatable version of Main Command
  public void indexToPointRepeatable(double goalPosition) {
    if (!((Math.abs(goalPosition - indexMotor.getPosition())) < 0.01)) {
      this.targetPosition(goalPosition);
      SmartDashboard.putBoolean("Moving", true);
    } else {
      SmartDashboard.putBoolean("Moving", false);
    }
    if (indexMotor.getSpeed() > 0) {
      setIndexSpeed(0);
    }
  }

  public Command continuousIndex() {

    return this.startEnd(() -> spinBarrels(), () -> stop());
  }

  private void spinBarrels() {

    indexMotor.set(-1 * barrelRotationSpeedModifier.getAsDouble());
  }

  private boolean atZero() {

    if (!hallEffectSensor.get()) {
      // System.out.println(rightAscensionMotor.getPosition());
      return true;
    }
    return false;
  }

  public void setIndexSpeed(double speed) {
    if (true) {
      indexMotor.set(speed);
    }
  }

  public void setIndexVoltage(Measure<Voltage> volts) {
    indexMotor.setVoltage(volts.magnitude());
  }

  private void stop() {
    indexMotor.set(0);
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
    // System.out.println("Indexing: " + hallEffectSensor.get());
  }
}
