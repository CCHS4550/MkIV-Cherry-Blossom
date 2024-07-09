package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.OI;
import java.text.DecimalFormat;

public class AimSimulator extends SubsystemBase {
  double xPos;
  double yPos;
  double xAngle;
  double yAngle;

  public AimSimulator() {
    xPos = 0;
    yPos = 0;
    xAngle = 0;
    yAngle = 0;
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
    xAngle = OI.normalize(xAngle, -Math.PI, Math.PI);
    return xAngle;
  }

    public double changeYAngle(double rad) {
    yAngle += rad;
    yAngle = OI.normalize(yAngle, 0, Math.PI/2);
    return yAngle;
  }

  public void printXY() {
    DecimalFormat df = new DecimalFormat("#.##");
    System.out.println("XPos: " + df.format(xPos) + "\tYPos: " + df.format(yPos));
  }

  public void printXYAngle() {
    DecimalFormat df = new DecimalFormat("#.##");
    System.out.println("XAngle: " + df.format(xAngle) + "\tYAngle: " + df.format(yAngle));
  }

  public void zeroXY() {
    xPos = 0;
    yPos = 0;
  }

  public void resetXYAngle() {
    xAngle = 0;
    yAngle = 0;
  }

  public void getClosestXAngle(double targetRad, double currentrad) {
    targetRad %= Math.PI;
  }

  @Override
  public void periodic() {
    // printXY();
  }
}
