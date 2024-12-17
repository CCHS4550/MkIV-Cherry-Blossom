// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.Vision;
import frc.maps.Constants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase implements Vision {

  public static PhotonVision mInstance;

  public static PhotonVision getInstance() {
    if (mInstance == null) {
      mInstance = new PhotonVision();
    }
    return mInstance;
  }

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  /* Create Camera */
  public PhotonCamera camera1;
  /* Camera 1 PhotonPoseEstimator. */
  public PhotonPoseEstimator camera1_photonPoseEstimator;

  private Pose2d lastEstimate = new Pose2d();

  /** Creates a new Vision. */
  private PhotonVision() {

    PortForwarder.add(5800, "photonvision.local", 5800);

    camera1 = new PhotonCamera(Constants.cameraOne.CAMERA_ONE_NAME);
    camera1_photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.cameraOne.ROBOT_TO_CAM);
    camera1_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return camera1_photonPoseEstimator.update();
  }

  /**
   * This specific overriden method is only for one camera, camera1. IMPORTANT METHOD! Main method
   * for updating PhotonVision Data!
   *
   * @param VisionData - This is a container object that stores all the data surrounding Vision.
   *     More information in Vision.java
   * @param currentEstimate - This is where the robot thinks it is at this moment, before it updates
   *     itself through the SwerveDrivePoseEstimator.
   */
  @Override
  public void updateInputs(VisionData visionData, Pose2d currentEstimate) {
    lastEstimate = currentEstimate;

    /* Only an array in case we use multiple cameras. */
    PhotonPipelineResult[] results = new PhotonPipelineResult[] {getLatestResult(camera1)};

    /* Only an array in case we use multiple cameras. */
    PhotonPoseEstimator[] photonEstimators =
        new PhotonPoseEstimator[] {camera1_photonPoseEstimator};

    // Resetting the poseEstimates every period?
    visionData.poseEstimates = new ArrayList<Pose2d>();
    visionData.poseEstimates.add(new Pose2d());

    visionData.timestamp = estimateLatestTimestamp(results);

    /** If you have a target, then update the poseEstimate ArrayList to equal that. */
    if (hasPoseEstimation(results)) {
      // inputs.results = results;
      visionData.poseEstimates = getPoseEstimatesArrayCondensed(results, photonEstimators);
      visionData.hasEstimate = true;

      int[][] cameraTargets = new int[][] {visionData.camera1Targets};
      visionData.camera1Targets = cameraTargets[0];
    } else {
      visionData.timestamp = visionData.timestamp;
      visionData.hasEstimate = false;
    }
  }

  /**
   * These methods below are kinda useless unless you have multiple cameras. check this out:
   * https://github.com/FRC1257/2024-Robot/blob/master/src/main/java/frc/robot/subsystems/vision/VisionIOPhoton.java#L152
   */
  // private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {

  //       camera1_photonPoseEstimator.setReferencePose(currentEstimate);

  //       return new PhotonPoseEstimator[] { camera1_photonPoseEstimator };
  //   }

  // private PhotonPipelineResult[] getAprilTagResults() {

  //       PhotonPipelineResult camera1_result = getLatestResult(camera1);

  //       // printStuff("cam1", cam1_result);

  //       return new PhotonPipelineResult[] { camera1_result };
  //   }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
