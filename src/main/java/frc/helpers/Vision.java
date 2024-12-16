// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public interface Vision {

        public static class VisionIOInputs {
    public Pose2d[] estimate = new Pose2d[0];
    public double timestamp = 0;
    public double[] timestampArray = new double[0];

    public int[] camera1Targets = new int[0];
    public int[] camera2Targets = new int[0];
    public int[] camera3Targets = new int[0];

    public boolean hasEstimate = false;

    // public byte[] results;
  }

  /** Helper Class */
  public default PhotonPipelineResult getLatestResult(PhotonCamera camera) {
    return camera.getLatestResult();
  }

  /** Only needed if there are multiple cameras.
   * @param results - An array made in the PhotonVision class compiling all the PhotonPipeline results
   * @param photonEstimator - The singular PhotonPoseEstimator instance.
   */
  public default Optional<Pose2d>[] getEstimates(PhotonPipelineResult[] results,
      PhotonPoseEstimator[] photonEstimator) {
    ArrayList<Optional<Pose2d>> estimates = new ArrayList<>();


    for (int i = 0; i < results.length; i++) {
    /**  */
      PhotonPipelineResult result = results[i];

      if (result.hasTargets()) {

        var est = photonEstimator[i].update();
        /** If the value exists and double checking if it has a target. */
        if (est.isPresent() && goodResult(result)) {
            /** Add the estimatedPose (Pose3d) object of the EstimatedRobotPose to the estimates array.  */
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
    /** Returns a Pose2d Array, of every estimated position from each of the cameras. Size depends */
    return estimatesArray;
}


  public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {
  }

  public default boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets();
  }

}
