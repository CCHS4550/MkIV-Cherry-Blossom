// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.maps.Constants;
import java.util.function.DoubleSupplier;
import frc.commands.defaultcommands.ReloadingDefault;

public class Reloading extends SubsystemBase {
  PneumaticsSystem pneumatics;
  DoubleSupplier barrelRotationSpeedModifier = () -> 0.5;
  Double barrelAngle;

  private DigitalInput hallEffectSensor =
      new DigitalInput(Constants.SensorMiscConstants.BARREL_SENSOR);

  // CCSparkMax barrelRotationMotor = new CCSparkMax("barrelRotationMotor","bRM", 13,
  // MotorType.kBrushless, IdleMode.kBrake, false);

  // Will not work because PWM is only an output!
  // private PWM hallEffectSensor = new PWM(0);

  /** Creates a new BarrelRotation. */
  public Reloading(PneumaticsSystem pneumatics) {
    this.pneumatics = pneumatics;

    setDefaultCommand(new ReloadingDefault(this));

  }

  public Command reload() {

    return this.runEnd(
        () -> {
          spinBarrels();
        },
        () -> {});
  }

  private void spinBarrels() {
    // barrelRotationMotor.set(1 * barrelRotationSpeedModifier.getAsDouble());
  }

  private void checkZero() {
    // returns value 0-1
    // double hallEffectInput = MathUtil.applyDeadband(hallEffectSensor.getPosition(), 0.1);
  }

  private void rotateUntilReady() {
    // TODO: need to find how many rotations of motor corresponds to full barrel rotation

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
