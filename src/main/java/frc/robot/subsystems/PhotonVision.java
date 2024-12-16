// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.Vision;
import frc.maps.Constants;

public class PhotonVision extends SubsystemBase implements Vision {

  public static PhotonVision mInstance;

	public static PhotonVision getInstance() {
		if (mInstance == null) {
			mInstance = new PhotonVision();
		} 
		return mInstance;
	}


  public PhotonCamera camera1;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  

  /** The singular PhotonPoseEstimator. */
  public PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.cameraOne.ROBOT_TO_CAM);
  
  
  /** Creates a new Vision. */
  public PhotonVision() {

    PortForwarder.add(5800, "photonvision.local", 5800);

    camera1 = new PhotonCamera(Constants.cameraOne.CAMERA_ONE_NAME);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return m_photonPoseEstimator.update();
    }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
