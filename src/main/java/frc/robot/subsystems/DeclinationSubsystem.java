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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

public class DeclinationSubsystem extends SubsystemBase {

  AimSimulator aimer;
  double declinationSpeedModifier = 0.1;

  ArmFeedforward declinationFeedForward =
      new ArmFeedforward(
          Constants.FeedForwardConstants.DECLINATION_KS,
          Constants.FeedForwardConstants.DECLINATION_KG,
          Constants.FeedForwardConstants.DECLINATION_KV,
          Constants.FeedForwardConstants.DECLINATION_KA);

  /*
   * This is the controller that actually brings the mechanism to the point.
   * Very important to test manually! Google PID tuning to find out how to tune PID constants.
   */
  PIDController declinationFeedBack = new PIDController(.9, 0.2, 0.00);

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
          // (2 * Math.PI) * 0.0125,
          // ((2 * Math.PI) * 0.0125 * 0.0166));
          1,
          1);

  private CCSparkMax declination2 =
      new CCSparkMax(
          "pitchMotor2",
          "pM2",
          Constants.MotorConstants.DECLINATION[1],
          MotorType.kBrushless,
          IdleMode.kCoast,
          true,
          // ((Math.PI * 2) * 0.0125),
          // ((Math.PI * 2) * 0.0125 * 0.0166));
          1,
          1);

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
              (voltage) -> setPitchVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  /** Creates a new Declination. */
  public DeclinationSubsystem(AimSimulator aimer) {

    this.aimer = aimer;

    declination1.setPosition(0);
    declination2.setPosition(0);

    /*
     * Test mechanism and view Sparkmax-##/MotorCurrentAmps.
     * If the mechanism draws more amps from hypothetically reaching an obstacle, the current will not surpass this and break the mechanism.
     */
    declination1.setSmartCurrentLimit(45);
    declination2.setSmartCurrentLimit(45);

    constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
    profile = new TrapezoidProfile(constraints);
    setPoint = new TrapezoidProfile.State();
    goal = new TrapezoidProfile.State();

    setSetpoint(new State(0, 0));
  }

  public void declinationSetUpDown(boolean isUp) {
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
    } else {
      return false;
    }
  }

  public double getDeclination() {
    return declination1.getPosition();
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
    double feedBackPower =
        declinationFeedBack.calculate(declination1.getPosition(), targetPosition);

    declination1.setVoltage(feedForwardPower + feedBackPower);
    declination2.setVoltage(feedForwardPower + feedBackPower);
    SmartDashboard.putNumber("feedForward + feedBack", (feedForwardPower + feedBackPower));
    // Sets the current setpoint to the point it will be in the future to prepare
    // for the next time
    // targetPosition() is called.
    setSetpoint(nextSetpoint);
  }

  // Main command called
  public Command declinationToPoint(double goalPosition) {
    return this.runEnd(() -> this.targetPosition(goalPosition), () -> setPitchSpeed(0));
    // .until(() -> ((Math.abs(goalPosition - declination1.getPosition())) < 0.01));
  }

  // Repeatable version of Main Command
  public void declinationToPointRepeatable(double goalPosition) {
    if (!((Math.abs(goalPosition - declination1.getPosition())) < 0.01)) {
      this.targetPosition(goalPosition);
      SmartDashboard.putBoolean("Moving", true);
    } else {
      SmartDashboard.putBoolean("Moving", false);
    }
    if (declination1.getSpeed() > 0) {
      setPitchSpeed(0);
    }
  }

  public void setPitchVoltage(Measure<Voltage> volts) {
    declination1.setVoltage(volts.magnitude());
    declination2.setVoltage(volts.magnitude());
  }

  public void setPitchSpeed(double speed) {
    if (!limitSwitchPressed()) {
      declination1.set(speed);
      declination2.set(speed);
    } else {
      if (speed > 0) {
        speed *= -1;
      }
      declination1.set(speed);
      declination1.set(speed);
    }
  }

  public void printEncoders() {
    System.out.println("Declination:" + declination1.getPosition());
  }

  public void declinationStop() {
    declination1.set(0);
    declination2.set(0);
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

  public static double convertDeclination(double degree) {

    // convert to a percentage
    degree /= 360;
    // 4:1 gearbox
    // 20:1 rack
    degree *= 80;

    return degree;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // printEncoders();
    // System.out.println("1:" + declination1.getPosition());
    // System.out.println("2:" + declination2.getPosition());
    // System.out.println(pitchLimitSwitch.get());
    // System.out.println();
    SmartDashboard.putNumber("Y Goal", getGoal().position);
    SmartDashboard.putNumber("Y Actual", declination1.getPosition());
    declinationToPointRepeatable(aimer.yAngle);
  }
}
