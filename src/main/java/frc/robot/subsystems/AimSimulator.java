package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.OI;

public class AimSimulator extends SubsystemBase{
    double xPos;
    double yPos;

    public AimSimulator() {
        xPos = 0;
        yPos = 0;
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

    public void printXY() {
        DecimalFormat df = new DecimalFormat("#.##");
        System.out.println("XPos: " + df.format(xPos) + "\tYPos: " + df.format(yPos));
    }

    public void zeroXY() {
        xPos = 0;
        yPos = 0;
    }


    @Override
    public void periodic() {
        printXY();
    }
}
