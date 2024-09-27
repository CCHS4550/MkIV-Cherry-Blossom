package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AimSimulator;
import frc.robot.subsystems.DeclinationSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
// import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.RightAscensionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class MechanismScheme {
  CommandXboxController primaryController;
  CommandXboxController sencondaryController;

  public static void configure(
      SwerveDrive swerveDrive,
      IndexingSubsystem indexer,
      DeclinationSubsystem declination,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension,
      CommandXboxController primaryController,
      CommandXboxController secondaryController,
      AimSimulator aimer) {

    RunCommand defaultDeclination =
        new RunCommand(() -> declination.declinationToPointRepeatable(aimer.yAngle), declination);
    RunCommand defaultRightAscension =
        new RunCommand(
            () -> rightAscension.rightAscensionToPointRepeatable(aimer.xAngle), rightAscension);
    RunCommand defaultIndex =
        new RunCommand(() -> indexer.indexToPointRepeatable(aimer.barrelAngle), indexer);

    declination.setDefaultCommand(defaultDeclination);
    rightAscension.setDefaultCommand(defaultRightAscension);
    // indexer.setDefaultCommand(defaultIndex);

    configureButtons(
        swerveDrive,
        indexer,
        declination,
        pneumatics,
        rightAscension,
        primaryController,
        secondaryController,
        aimer);
  }

  public static void configureButtons(
      SwerveDrive swerveDrive,
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

    /*
     * Toggle air compressors and fan with left trigger. (FUNCTIONAL)
     */
    controller.leftTrigger().onTrue(new InstantCommand(() -> pneumatics.toggleAirCompressors()));

    /*
     * Continuously rotates the barrels. (FUNCTIONAL)
     */
    // controller.rightTrigger().whileTrue(aimer.continuousBarrelChange(Math.toRadians(1)));
    // controller.rightTrigger().onTrue(indexer.indexAllDemo());

    /*
     * TODO make this work
     */
    controller.y().onTrue(new InstantCommand(() -> pneumatics.togglePressureSeal()));

    controller.rightBumper().and(controller.leftBumper()).onTrue(indexer.shootAll());

    // controller.leftBumper().onTrue(new InstantCommand(() -> System.out.println("ajdsfdlgkfh")));
    // controller.a().onTrue(new InstantCommand(() -> rightAscension.zeroEncoders()));

    /*
     * Resets the Aimsimulator, resetting the heading.
     */
    controller2.a().onTrue(new InstantCommand(() -> aimer.zeroXYAngle()));

    /*
     * Ignore this
     */
    // controller.povUp().whileTrue(aimer.continuousYChange(Math.toRadians(5)));
    // controller.povDown().whileTrue(aimer.continuousYChange(-Math.toRadians(5)));
    // controller.povLeft().whileTrue(aimer.continuousXChange(Math.toRadians(5)));
    // controller.povRight().whileTrue(aimer.continuousXChange(-Math.toRadians(5)));
    // controller.povUp().onTrue(new InstantCommand(() -> System.out.println("adsjfgkl")));

    /*
     * Default controls.
     */
    // controller.povUp().whileTrue(aimer.continuousYAngleChange(0.0175));
    // controller.povDown().whileTrue(aimer.continuousYAngleChange(-0.0175));

    // controller
    //     .povUp()
    //     .whileTrue(aimer.continuousYAngleChange(DeclinationSubsystem.convertDeclination(1)));
    // controller
    //     .povDown()
    //     .whileTrue(aimer.continuousYAngleChange(-DeclinationSubsystem.convertDeclination(1)));

    // controller
    //     .povLeft()
    //
    // .whileTrue(aimer.continuousXAngleChange(RightAscensionSubsystem.convertRightAscension(-1)));
    // controller
    //     .povRight()
    //
    // .whileTrue(aimer.continuousXAngleChange(RightAscensionSubsystem.convertRightAscension(1)));

    controller.povUp().whileTrue(aimer.continuousYAngleChange(Math.toRadians(1)));
    controller.povDown().whileTrue(aimer.continuousYAngleChange(-Math.toRadians(1)));

    controller.povLeft().whileTrue(aimer.continuousXAngleChange(-Math.toRadians(1)));
    controller.povRight().whileTrue(aimer.continuousXAngleChange(Math.toRadians(1)));

    /*
     * Tells rightAscension and declination to move to a specific point for testing.
     */

    //   controller.povUp().onTrue(declination.declinationToPoint(5));
    //   controller.povDown().onTrue(declination.declinationToPoint(3));
    //   controller.povLeft().onTrue(rightAscension.rightAscensionToPoint(-5));
    //   controller.povRight().whileTrue(rightAscension.rightAscensionToPoint(5));
  }

  public static void configureTeleopPeriodic() {}
}
