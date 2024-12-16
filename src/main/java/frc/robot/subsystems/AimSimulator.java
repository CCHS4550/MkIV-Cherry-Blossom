package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.OI;
import java.text.DecimalFormat;

public class AimSimulator extends SubsystemBase {

  
  public static AimSimulator mInstance;

	public static AimSimulator getInstance() {
		if (mInstance == null) {
			mInstance = new AimSimulator();
		} 
		return mInstance;
	}
  public double xPos;
  public double yPos;
  public double xAngle;
  public double yAngle;

  public double barrelAngle;

  public AimSimulator() {
    xPos = 0;
    yPos = 0;
    xAngle = 0;
    yAngle = 0;
    barrelAngle = 0;
  }

  public double changeXPos(double val) {
    xPos += val;
    xPos = OI.normalize(xPos, -1, 1);
    return xPos;
  }

  public double changeYPos(double val) {
    yPos += val;
    yPos = OI.normalize(yPos, -1, 1);
    return yPos;
  }

  public double changeXAngle(double rad) {
    xAngle += rad;
    // xAngle = OI.normalize(xAngle, -Math.PI, Math.PI);
    return xAngle;
  }

  public double changeYAngle(double rad) {
    yAngle += rad;
    yAngle = OI.normalize(yAngle, 0, 1);
    return yAngle;
  }

  public void printXY() {
    DecimalFormat df = new DecimalFormat("#.##");
    System.out.println("XPos: " + df.format(xPos) + "\tYPos: " + df.format(yPos));
  }

  public void printXYAngle() {
    // DecimalFormat df = new DecimalFormat("#.##");
    // System.out.println("XAngle: " + df.format(xAngle) + "\tYAngle: " + df.format(yAngle));
    //   SmartDashboard.putNumber("xAngle", xAngle);
    //   SmartDashboard.putNumber("yAngle", yAngle);
  }

  public void zeroXY() {
    xPos = 0;
    yPos = 0;
  }

  public void zeroXYAngle() {
    xAngle = 0;
    yAngle = 0;
  }

  public void getClosestXAngle(double targetRad, double currentrad) {
    targetRad %= Math.PI;
  }

  public Command continuousXChange(double increment) {
    return this.run(() -> changeXPos(increment));
  }

  public Command continuousYChange(double increment) {
    return this.run(() -> changeYPos(increment));
  }

  public Command continuousXAngleChange(double increment) {
    return this.run(() -> changeXAngle(increment));
  }

  public Command continuousYAngleChange(double increment) {
    return this.run(() -> changeYAngle(increment));
  }

  public double changeBarrelAngle(double rad) {
    barrelAngle += rad;
    // barrelAngle = OI.normalize(barrelAngle, 0, 11);
    return barrelAngle;
  }

  public Command continuousBarrelChange(double increment) {
    return this.run(() -> changeBarrelAngle(increment));
  }

  @Override
  public void periodic() {
    // printXY();
    // printXYAngle();
    // SmartDashboard.putNumber("barrel Goal", barrelAngle);
  }
}
