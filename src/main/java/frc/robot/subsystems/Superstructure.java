// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  public enum WantedSuperState {
    IDLE_STATIC,
    IDLE_TRACKING,
    RELOAD,
    INDEX,
    SHOOT,
    VISION_TRACKING_DEMO,
    SHOW_OFF
  }

  public enum CurrentSuperState {
    IDLE_STATIC,
    IDLE_TRACKING,
    RELOAD,
    INDEX,
    SHOOT,
    VISION_TRACKING_DEMO,
    SHOW_OFF
  }

  private WantedSuperState wantedSuperState = WantedSuperState.IDLE_STATIC;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLE_STATIC;

  private DeclinationSubsystem declination;
  private IndexingSubsystem indexer;
  private Lights lights;
  private PneumaticsSystem pneumatics;
  private RightAscensionSubsystem rightAscension;

  /** Creates a new Superstructure. */
  public Superstructure(
      DeclinationSubsystem declination,
      IndexingSubsystem indexer,
      Lights lights,
      PneumaticsSystem pneumatics,
      RightAscensionSubsystem rightAscension) {
    this.declination = declination;
    this.indexer = indexer;
    this.lights = lights;
    this.pneumatics = pneumatics;
    this.rightAscension = rightAscension;
  }

  @Override
  public void periodic() {

    currentSuperState = handleStateTransitions();
    applyStates();

    // This method will be called once per scheduler run
  }

  private void applyStates() {
    switch (currentSuperState) {
      case IDLE_STATIC:
        idleStatic();
        break;
      case IDLE_TRACKING:
        break;
      case RELOAD:
        break;
      case INDEX:
        break;
      case SHOOT:
        break;
    }
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case IDLE_STATIC:
        currentSuperState = CurrentSuperState.IDLE_STATIC;

      case IDLE_TRACKING:
        currentSuperState = CurrentSuperState.IDLE_STATIC;

      case RELOAD:
        currentSuperState = CurrentSuperState.RELOAD;

      case INDEX:
        currentSuperState = CurrentSuperState.INDEX;
    }

    return currentSuperState;
  }

  private void idleStatic() {}
}
