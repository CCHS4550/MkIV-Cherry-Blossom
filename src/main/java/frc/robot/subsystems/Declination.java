// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.maps.Constants;

public class Declination extends SubsystemBase {

  double declinationSpeedModifier = 0.1;

  double pitchLocation;
  double pitchOffset1;
  double pitchOffset2;

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
  double distanceFromEdge = 1 - (Math.abs(pitchLocation - middlePoint) / (range /2));
  double maxOutwardSpeed  =  distanceFromEdge / 1;




  private DigitalInput pitchLimitSwitch = new DigitalInput(Constants.SensorMiscConstants.PITCH_LIMIT_SWITCH);
  
  private CCSparkMax declination1 = new CCSparkMax
  (
    "pitchMotor1",
    "pM1",
    Constants.MotorConstants.DECLINATION[0],
    MotorType.kBrushless,
    IdleMode.kCoast,
    false,
    (2 * Math.PI) / 7,
    ((2 * Math.PI) / 7) / 60
  );

  private CCSparkMax declination2 = new CCSparkMax
  (
    "pitchMotor2",
    "pM2",
    Constants.MotorConstants.DECLINATION[1],
    MotorType.kBrushless,
    IdleMode.kCoast,
    true,
    (2 * Math.PI) / 7,
    ((2 * Math.PI) / 7) / 60
  );

  

  

  /** Creates a new Declination. */
  public Declination() {

    pitchOffset1 = 0.0;
    pitchOffset2 = 0.0;
    declination1.setPosition(0);
    declination2.setPosition(0);
  }

  public void declinationDefaultMethod(CommandXboxController controller) {
    double declinationSpeed = MathUtil.applyDeadband(-controller.getRightY(), 0.05) * declinationSpeedModifier;

    System.out.println(pitchLimitSwitch.get());
    // if (pitchLimitSwitch.get() && pitchLocation < (Math.PI/2)) {
      declination2.set
      (
      
        declinationSpeed
        // this.normalizeSpeed(declinationSpeed, pitchLocation)
      );
    // }

    this.checkZeroPitch();

    // Calculated in radians
    pitchLocation = Math.abs(declination1.getPosition()) - pitchOffset1;

  }

  private void checkZeroPitch(){
    
    if (!pitchLimitSwitch.get()) {


      // CCSparkMax.java makes it so that .getPosition() returns a value in Radians
      pitchOffset1 = Math.abs(((declination1.getPosition() / 126) % 1) * (2 * Math.PI));
      pitchOffset2 = Math.abs(((declination2.getPosition() / 126) % 1) * (2 * Math.PI));
      
      
    }
  }

  public double getDeclination(){
    return pitchLocation;
  }

  private boolean pastMidpoint(double location){
    if (location > middlePoint) {
      return true;
    } else {
      return false; 
    }
  }
    
  private double normalizeSpeed(double speed, double location){
    if (speed > 0 && this.pastMidpoint(location)) {
      return OI.normalize(speed, 0, maxOutwardSpeed);
    } else if (speed < 0 && !this.pastMidpoint(location)) {
      return OI.normalize(speed, -maxOutwardSpeed, 0);
    } else {
      return speed;
    }
  } 

  public void printEncoders() {
    System.out.println("Declination:" + declination1.getPosition());
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // printEncoders();
  }
}
