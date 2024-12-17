// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

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

  /** Helper Class */
  public default PhotonPipelineResult getLatestResult(PhotonCamera camera) {
    return camera.getLatestResult();
  }

  /**
   * Only needed if there are multiple cameras.
   *
   * @param results - An array made in the PhotonVision class compiling all the PhotonPipeline
   *     results
   * @param photonEstimator - The multiple PhotonPoseEstimator instances for each camera.
   */
  public default Optional<Pose2d>[] getPoseEstimateArray(
      PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {
    ArrayList<Optional<Pose2d>> estimates = new ArrayList<>();

    for (int i = 0; i < results.length; i++) {
      /** Isolate single result */
      PhotonPipelineResult result = results[i];

      /** Check if single result has a target */
      if (result.hasTargets()) {
        /** Update Pose Estimator for this camera. */
        var est = photonEstimator[i].update();
        /** If the value exists and double checking if it has a target. */
        if (est.isPresent() && goodResult(result)) {
          /**
           * Add the estimatedPose (Pose3d) object of the EstimatedRobotPose to the estimates array.
           */
          estimates.add(Optional.of(est.get().estimatedPose.toPose2d()));
        } else {
          /** If there is nothing, add an empty object */
          estimates.add(Optional.empty());
        }
      } else {
        estimates.add(Optional.empty());
      }
    }

    Optional<Pose2d>[] estimatesArray = estimates.toArray(new Optional[0]);
    /**
     * Returns a Pose2d Array, of every estimated position from each of the cameras. Size depends
     */
    return estimatesArray;
  }

  /** Change Optional<Pose2d[]> to List<Pose2d>, taking out the Optional.empty() */
  /**
   * Takes PhotonPipelineResults and a PhotonPoseEstimator Factory and pumps out an ArrayList with
   * the estimated Poses.
   *
   * @param results - Raw results gotten from the camera, through the getLatestResult() method.
   * @param photonEstimator - An array of pose estimators that match their corresponding pipeline
   *     result.
   * @return An ArrayList with Pose2d objects.
   */
  public default List<Pose2d> getPoseEstimatesArrayCondensed(
      PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {

    Optional<Pose2d>[] estimates = getPoseEstimateArray(results, photonEstimator);

    List<Pose2d> finalEstimates = new ArrayList<>();
    for (int i = 0; i < estimates.length; i++) {
      if (estimates[i].isPresent()) {
        finalEstimates.add(estimates[i].get());
      }
    }

    return finalEstimates;
  }

  public default void updateInputs(VisionData inputs, Pose2d estimate) {}

  public default boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets();
    //  && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD;
  }

  public default double estimateLatestTimestamp(PhotonPipelineResult[] results) {
    double latestTimestamp = 0;
    int count = 0;
    for (PhotonPipelineResult result : results) {
      latestTimestamp = result.getTimestampSeconds();
      count++;
    }
    return latestTimestamp / count;
  }

  public default double[] getTimestampArray(PhotonPipelineResult[] results) {
    double[] timestamps = new double[results.length];
    for (int i = 0; i < results.length; i++) {
      timestamps[i] = results[i].getTimestampSeconds();
    }
    return timestamps;
  }

  public default boolean hasPoseEstimation(PhotonPipelineResult[] results) {
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        return true;
      }
    }
    return false;
  }
}
