// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */

/** Helper Class! Well actually, helper interface. */
public interface Vision {

  //   @AutoLog - for some reason autolog not working!

  /** This is a container class to contain all the vision data. */
  public static class VisionData {
    public List<Pose2d> poseEstimates = new ArrayList<>();
    public double timestamp = 0;
    public double[] timestampArray = new double[0];

    public int[] camera1Targets = new int[0];
    // public int[] camera2Targets = new int[0];
    // public int[] camera3Targets = new int[0];

    public boolean hasEstimate = false;

    // public byte[] results;
  }

  /** Default method, defined in photonvision */
  public default void updateData(VisionData inputs, Pose2d estimate) {}
}
