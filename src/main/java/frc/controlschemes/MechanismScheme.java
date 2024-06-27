package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BarrelRotation;
import frc.robot.subsystems.Declination;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscension;

public class MechanismScheme {


    public static void configure(BarrelRotation barrelRotation, Declination declination, PneumaticsSystem pneumatics, RightAscension rightAscension, CommandXboxController controller){




        rightAscension.setDefaultCommand(new RunCommand(() -> {
            rightAscension.rightAscensionDefaultMethod(controller);
        }, rightAscension));

        // declination.setDefaultCommand(new RunCommand(() -> {
        //     declination.declinationDefaultMethod(controller);
        // }, declination));


        configureButtons(barrelRotation, declination, pneumatics, rightAscension, controller);
    }

    public static void configureButtons(BarrelRotation barrelRotation, Declination declination, PneumaticsSystem pneumatics, RightAscension rightAscension, CommandXboxController controller){

        // Toggle air compressors and fan with left trigger.
        controller.leftTrigger().onTrue(pneumatics.toggleAirCompressors());
        // Rotate Barrels
        controller.rightTrigger().whileTrue(barrelRotation.reload());
        // Shoot the cannon by pressing both controller bumpers.
        controller.leftBumper().and(controller.rightBumper()).onTrue(pneumatics.shoot());
        controller.a().onTrue(new InstantCommand(() -> rightAscension.zeroEncoders()));
    }
    
}
