package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase{
    private CCSparkMax driveMotor;
    private CCSparkMax turnMotor;

    private PIDController turnPIDController;
    private PIDController drivePIDController;
    private SimpleMotorFeedforward driveFeedForward;

    private AnalogEncoder absoluteEncoder;
    private double absoluteEncoderOffset;
    private boolean absoluteEncoderReversed;
    private int absoluteEncoderPort;

    private String name;

    public SwerveModule (CCSparkMax driveMotor, CCSparkMax turnMotor,  double absoluteEncoderOffset, int absoluteEncoderPort, String name){
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        absoluteEncoder = new AnalogEncoder (absoluteEncoderPort);
        absoluteEncoder.setDistancePerRotation(2*Math.pi); 
        turnPIDController = new PIDContoller (0.5,0,0);
        turnPIDController.enableContinuousInput(0, 2*Math.pi);
        drivePIDController = new PIDController (1,0,0);

        
    }

}