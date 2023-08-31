// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.models.AprilTagTracker;

public class VisionTrackingSubsystem extends SubsystemBase {

  private final List<AprilTagTracker> aprilTagTrackers = new ArrayList<>();

  PhotonCamera photonCamera;
  PhotonPoseEstimator photonPoseEstimator;

  public VisionTrackingSubsystem() {

    // Change the name of your camera here to whatever it is in the PhotonVision UI.
    photonCamera = new PhotonCamera(VisionConstants.cameraName);

    // Initialize the AprilTagTrackers
    aprilTagTrackers.add(new AprilTagTracker("1", "6"));
    aprilTagTrackers.add(new AprilTagTracker("2", "7"));
    aprilTagTrackers.add(new AprilTagTracker("3", "8"));

    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
      // on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

      // Create pose estimator
      photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera,
          VisionConstants.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
   *         targets used to create
   *         the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void updateTagDetection() {
    // This is where you would update the isDetected flags of your AprilTagTrackers
    // based on whatever AprilTag detection logic you're using.
    // For example, you might query the photonCamera for detected tags and then
    // update the corresponding AprilTagTracker's isDetected flag.
  }

  @Override
  public void periodic() {
    updateTagDetection();

    // Display the detection status of the AprilTags on the SmartDashboard
    for (int i = 0; i < aprilTagTrackers.size(); ++i) {
      AprilTagTracker tracker = aprilTagTrackers.get(i);
      SmartDashboard.putBoolean("AprilTag " + (i + 1), tracker.isDetected);
    }
  }
}
