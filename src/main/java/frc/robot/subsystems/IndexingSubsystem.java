// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commands.defaultcommands.ReloadingDefault;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import java.util.function.DoubleSupplier;

public class IndexingSubsystem extends SubsystemBase {
  PneumaticsSystem pneumatics;
  DoubleSupplier barrelRotationSpeedModifier = () -> 1;
  Double barrelAngle;

  private DigitalInput hallEffectSensor =
      new DigitalInput(Constants.SensorMiscConstants.BARREL_SENSOR);

  CCSparkMax barrelRotationMotor =
      new CCSparkMax(
          "barrelRotationMotor", "bRM", 13, MotorType.kBrushless, IdleMode.kBrake, false);

  // Will not work because PWM is only an output!
  // private PWM hallEffectSensor = new PWM(0);

  /** Creates a new IndexingSubsystem. */
  public IndexingSubsystem(PneumaticsSystem pneumatics) {
    this.pneumatics = pneumatics;

    setDefaultCommand(new ReloadingDefault(this));
  }

  public Command continuousIndex() {

    return this.startEnd(() -> spinBarrels(), () -> stop());
  }

  private void spinBarrels() {

    barrelRotationMotor.set(-1 * barrelRotationSpeedModifier.getAsDouble());
  }

  private void stop() {
    barrelRotationMotor.set(0);
  }

  private boolean atZero() {

    if (!hallEffectSensor.get()) {
      // System.out.println(rightAscensionMotor.getPosition());
      return true;
    }
    return false;
  }

  private void rotateUntilReady() {
    // TODO: need to find how many rotations of motor corresponds to full barrel rotation

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Indexing: " + hallEffectSensor.get());
  }
}
