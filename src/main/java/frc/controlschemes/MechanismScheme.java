package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
// import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;

public class MechanismScheme {

  public static void configure(
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController primaryController,
      CommandXboxController secondaryController,
      AimSimulator aimer) {

    configureButtons(
        indexer,
        declination,
        pneumatics,
        rightAscension,
        primaryController,
        secondaryController,
        aimer);
  }

  public static void configureButtons(
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController controller,
      CommandXboxController controller2,
      AimSimulator aimer) {

    // controller
    //     .povUp()
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> declination.declinationSetUpDown(true), () ->
    // declination.declinationStop()));

    // controller
    //     .povDown()
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> declination.declinationSetUpDown(false),
    //             () -> declination.declinationStop()));

    // controller.povDown().whileTrue(new DeclinationManual(declination));
    // Toggle air compressors and fan with left trigger.
    controller.leftTrigger().onTrue(new InstantCommand(() -> pneumatics.toggleAirCompressors()));
    // Rotate Barrels
    controller.rightTrigger().whileTrue(indexer.continuousIndex());
    // Shoot the cannon by pressing both controller bumpers.
    controller.leftBumper().onTrue(pneumatics.togglePressureSeal());
    controller.rightBumper().onTrue(pneumatics.shoot());
    // controller.leftBumper().onTrue(new InstantCommand(() -> System.out.println("ajdsfdlgkfh")));
    // controller.a().onTrue(new InstantCommand(() -> rightAscension.zeroEncoders()));

    controller2.a().onTrue(new InstantCommand(() -> aimer.zeroXY()));

    controller.povUp().whileTrue(aimer.continuousYChange(Math.toRadians(.25)));
    controller.povDown().whileTrue(aimer.continuousYChange(-Math.toRadians(.25)));
    controller.povLeft().whileTrue(aimer.continuousXChange(Math.toRadians(.25)));
    controller.povRight().whileTrue(aimer.continuousXChange(-Math.toRadians(.25)));
    // controller.povUp().onTrue(new InstantCommand(() -> System.out.println("adsjfgkl")));
  }
}
