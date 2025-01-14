// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.maps.Constants;
import frc.robot.autonomous.CustomAutoChooser;
import frc.robot.subsystems.Lights.LEDState;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private RobotContainer robotContainer;
  CustomAutoChooser autoChooser;

  private boolean browningOut = false;

  public Robot() {}

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    autoChooser = new CustomAutoChooser();

    Constants.getCurrentMode();

    SmartDashboard.putBoolean("Browning Out?", browningOut);

    // // Set up data receivers & replay source

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/U/logs")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher());
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    PortForwarder.add(5800, "photonvision.local", 5800);

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Unofficial REV-Compatible Logger
    // Used by SysID to log REV devices
    Logger.registerURCL(URCL.startExternal());
    // Start AdvantageKit logger
    Logger.start();

    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    switch (Constants.currentMode) {
      case REAL:
        RobotState.getInstance().updateDashboard();
        RobotState.getInstance().updateVisionPose();

        if (RobotController.getBatteryVoltage() < 10) {
          browningOut = true;
        } else {
          browningOut = false;
        }

        break;

      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);

        break;

      case REPLAY:
        break;
    }
    RobotState.getInstance().updateOdometryPose();
    RobotState.getInstance().updateModulePositions();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {

    // AutoBuilderScheme.getPathPlannerAutoCommand().schedule();
    // AutoBuilderScheme.getCustomAuto().schedule();
    autoChooser.getSelectedCustomCommand().schedule();

    System.out.println("Autonomous Routine Scheduled!");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotContainer
        .lights
        .multipleLightCommands(LEDState.pinkWhiteGradient, LEDState.pressureRedtoGreenGradient)
        .schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
