// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.maps.Constants;
import java.util.function.DoubleSupplier;

public class IndexingSubsystem extends SubsystemBase {

  PneumaticsSystem pneumatics;
  DoubleSupplier barrelRotationSpeedModifier = () -> 1;

  SimpleMotorFeedforward indexFeedForward =
      new SimpleMotorFeedforward(
          Constants.FeedForwardConstants.INDEX_KS,
          Constants.FeedForwardConstants.INDEX_KV,
          Constants.FeedForwardConstants.INDEX_KA);

  /*
   * This is the controller that actually brings the mechanism to the point.
   * Very important to test manually! Google PID tuning to find out how to tune PID constants.
   */
  PIDController indexFeedBack = new PIDController(4, 7, 0.4);

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

  /* 220:1 Gear Reduction */
  public CCSparkMax indexMotor =
      new CCSparkMax(
          "barrelRotationMotor",
          "bRM",
          Constants.MotorConstants.BARREL_ROTATION,
          MotorType.kBrushless,
          IdleMode.kCoast,
          true,
          ((2 * Math.PI) * (1 / 35.166)),
          ((2 * Math.PI) * (1 / 35.166) * 0.0166));
  // 1,
  // 1);

  double nextBarrel = indexMotor.getPosition() + (Math.PI / 3) + 0.0067;

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(2),
              Volts.of(2),
              Seconds.of(3),
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

    // indexMotor.setSmartCurrentLimit(5);

    constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
    profile = new TrapezoidProfile(constraints);
    setPoint = new TrapezoidProfile.State();
    goal = new TrapezoidProfile.State();

    setSetpoint(new State(0, 0));

    // Shuffleboard.getTab("Aimer").add("Indexer: PID Controller", indexFeedBack);
    SmartDashboard.putData("Indexer: PID Controller", indexFeedBack);
  }

  public Command indexTshirt() {
    return new SequentialCommandGroup(
        pneumatics.disablePressureSealCommand(),
        indexBarrel(),
        pneumatics.enablePressureSealCommand());
    // .withTimeout(3);
  }

  /**
   * Helper method called repeatedly for indexToPoint() Method.
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
    double feedForwardPower =
        indexFeedForward.calculate(nextSetpoint.position, nextSetpoint.velocity);

    // The Pid Calculation, calculating a voltage using the current position and the
    // goal position.
    double feedBackPower = indexFeedBack.calculate(indexMotor.getPosition(), targetPosition);

    double totalPower = feedForwardPower + feedBackPower;
    totalPower = OI.normalize(totalPower, -4, 4);

    // if (pneumatics.psi > 40) {
    indexMotor.setVoltage(totalPower);
    // }
    SmartDashboard.putNumber("Indexer: Total Power", (totalPower));
    // Sets the current setpoint to the point it will be in the future to prepare
    // for the next time
    // targetPosition() is called.
    setSetpoint(nextSetpoint);
  }

  // Main command called
  public Command indexToPoint(double goalPosition) {
    return this.runEnd(
            () -> {
              this.targetPosition(goalPosition);
              SmartDashboard.putNumber("Index: GoalPosition", goalPosition);
            },
            () -> {
              setIndexSpeed(0);
              // nextBarrel = indexMotor.getPosition() + (Math.PI / 3);
            })
        .until(() -> ((Math.abs(goalPosition - indexMotor.getPosition())) < 0.01));
  }

  // Main command called
  public Command indexBarrel() {
    return this.runEnd(
            () -> {
              this.targetPosition(nextBarrel);
              // SmartDashboard.putNumber("Index: GoalPosition", goalPosition);
            },
            () -> {
              setIndexSpeed(0);
              nextBarrel = indexMotor.getPosition() + (Math.PI / 3) + 0.0067;
            })
        .until(() -> ((Math.abs(nextBarrel - indexMotor.getPosition())) < 0.01));
  }

  // Repeatable version of Main Command
  public void indexToPointRepeatable(double goalPosition) {
    if (!((Math.abs(goalPosition - indexMotor.getPosition())) < 0.01)) {
      this.targetPosition(goalPosition);
      // SmartDashboard.putBoolean("Moving", true);
    } else {
      // SmartDashboard.putBoolean("Moving", false);
    }
    if (indexMotor.getSpeed() > 0) {
      setIndexSpeed(0);
    }
  }

  public Command dislodgeIndexer() {
    // if (Math.abs(indexMotor.getOutputCurrent()) > 10) {
    return sequence(
        new InstantCommand(() -> indexMotor.setVoltage(-6)),
        new WaitCommand(0.2),
        new InstantCommand(() -> indexMotor.setVoltage(0)));

    // }
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

  private Command disablePressureSeal() {
    pneumatics.disablePressureSeal();
    return new WaitCommand(0.01);
  }

  private Command enablePressureSeal() {
    pneumatics.enablePressureSeal();
    return new WaitCommand(0.01);
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
    SmartDashboard.putNumber("barrel Actual", indexMotor.getPosition());
    SmartDashboard.putNumber("barrel Goal (runEnd)", getGoal().position);
    SmartDashboard.putNumber("next Barrel", nextBarrel);
  }
}
