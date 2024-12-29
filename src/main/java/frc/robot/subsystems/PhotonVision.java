// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.Vision;
import frc.maps.Constants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
  public PhotonCamera frontCamera;

  VisionSystemSim visionSim;
  
  /* Camera 1 PhotonPoseEstimator. */
  public PhotonPoseEstimator frontCamera_photonEstimator;

  /** Creates a new Photonvision. */
  private PhotonVision() {

    PortForwarder.add(5800, "limelight2.local", 5800);

     frontCamera = new PhotonCamera(Constants.cameraOne.CAMERA_ONE_NAME);

    switch (Constants.currentMode) {
      case REAL:
       

        frontCamera_photonEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCamera,
            Constants.cameraOne.ROBOT_TO_CAM);
        break;

      case SIM:
        visionSim = new VisionSystemSim("main");
        // TargetModel targetModel = TargetModel.kAprilTag36h11;
        visionSim.addAprilTags(aprilTagFieldLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(62.5));
        cameraProp.setFPS(40);

        PhotonCameraSim cameraSim = new PhotonCameraSim(frontCamera, cameraProp);


        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-35), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        visionSim.addCamera(cameraSim, robotToCamera);

        break;

        
        

      case REPLAY:
        break;
    }

    
    frontCamera_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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
  public void updateData(VisionData visionData, Pose2d currentEstimate) {

    /* Only an array in case we use multiple cameras. */
    PhotonPipelineResult[] results = new PhotonPipelineResult[] {frontCamera.getLatestResult()};

    /* Only an array in case we use multiple cameras. */
    PhotonPoseEstimator[] photonEstimators =
        new PhotonPoseEstimator[] {frontCamera_photonEstimator};

    // Resetting the poseEstimates every period?
    visionData.poseEstimates = new ArrayList<Pose2d>();
    // visionData.poseEstimates.add(new Pose2d());

    visionData.timestamp = estimateAverageTimestamp(results);

    /** If you have a target, then update the poseEstimate ArrayList to equal that. */
    if (hasAnyTarget(results)) {
      // inputs.results = results;

      visionData.poseEstimates = getPoseEstimatesArray(results, photonEstimators);
      visionData.hasEstimate = true;

      int[][] cameraTargets = new int[][] {visionData.camera1Targets};
      visionData.camera1Targets = cameraTargets[0];
    } else {
      visionData.timestamp = visionData.timestamp;
      visionData.hasEstimate = false;
    }
  }

  /**
   * Only needed if there are multiple cameras, but used in this situation nonetheless.
   *
   * <p>Takes PhotonPipelineResults and a PhotonPoseEstimator object and pumps out an ArrayList with
   * the estimated Poses it can find with any targets it might have.
   *
   * @param results - Raw results gotten from the camera, through the getLatestResult() method.
   * @param photonEstimator - An array of pose estimators that match their corresponding pipeline
   *     result.
   * @return An ArrayList with Pose2d objects.
   */
  public List<Pose2d> getPoseEstimatesArray(
      PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {

    List<Pose2d> estimates = new ArrayList<>();

    for (int i = 0; i < results.length; i++) {

      Optional<EstimatedRobotPose> estimatedPose = photonEstimator[i].update();
      if (estimatedPose.isPresent()) {
        estimates.add(estimatedPose.get().estimatedPose.toPose2d());
      }
      estimates.removeIf(pose -> pose == null);
    }

    return estimates;
  }

  public List<PhotonTrackedTarget> getTargetsList(PhotonPoseEstimator photonEstimator) {
    return photonEstimator.update().get().targetsUsed;
  }

  public double estimateAverageTimestamp(PhotonPipelineResult[] results) {
    double latestTimestamp = 0;
    int count = 0;
    for (PhotonPipelineResult result : results) {
      latestTimestamp = result.getTimestampSeconds();
      count++;
    }
    return latestTimestamp / count;
  }

  public double[] getTimestampArray(PhotonPipelineResult[] results) {
    double[] timestamps = new double[results.length];
    for (int i = 0; i < results.length; i++) {
      timestamps[i] = results[i].getTimestampSeconds();
    }
    return timestamps;
  }

  /** If any of the results have targets, then return true. */
  public boolean hasAnyTarget(PhotonPipelineResult[] results) {
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        return true;
      }
    }
    return false;
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

    Logger.recordOutput("VisionData/HasTarget?", RobotState.getInstance().visionData.hasEstimate);

    // This method will be called once per scheduler run
  }
}
