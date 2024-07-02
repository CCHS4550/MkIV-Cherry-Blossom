package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.BarrelRotation;
import frc.robot.subsystems.Declination;
// import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscension;

public class MechanismScheme {


    public static void configure(BarrelRotation barrelRotation, Declination declination, PneumaticsSystem pneumatics, RightAscension rightAscension, CommandXboxController controller, AimSimulator aimer){

        CommandXboxController controller2 = new CommandXboxController(1);



        // rightAscension.setDefaultCommand(new RunCommand(() -> {
        //     rightAscension.rightAscensionDefaultMethod(controller);
        // }, rightAscension));

        // declination.setDefaultCommand(new RunCommand(() -> {
        //     declination.declinationDefaultMethod(controller);
        // }, declination));

        // lights.setDefaultCommand(new RunCommand(() -> {
        //     lights.lightsDefaultMethod(controller);
        // }, lights));

        aimer.setDefaultCommand(new RunCommand(() -> {
            aimer.changeXPos(controller2.getRightX() / 100);
            aimer.changeYPos(controller2.getRightY() / 100);
        }, aimer));

        configureButtons(barrelRotation, declination, pneumatics, rightAscension, controller, controller2, aimer);
    }

    public static void configureButtons(BarrelRotation barrelRotation, Declination declination, PneumaticsSystem pneumatics, RightAscension rightAscension, CommandXboxController controller, CommandXboxController controller2, AimSimulator aimer){

        // Toggle air compressors and fan with left trigger.
        controller.leftTrigger().onTrue(new InstantCommand(() -> pneumatics.toggleAirCompressors()) );
        // Rotate Barrels
        controller.rightTrigger().whileTrue(barrelRotation.reload());
        // Shoot the cannon by pressing both controller bumpers.
        controller.leftBumper().onTrue(pneumatics.shoot());
        controller.a().onTrue(new InstantCommand(() -> rightAscension.zeroEncoders()));

        controller2.a().onTrue(new InstantCommand(() -> aimer.zeroXY()));
    }
    
}
