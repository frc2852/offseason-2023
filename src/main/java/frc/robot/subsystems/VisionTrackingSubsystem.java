// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Standard Java imports
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

// PhotonVision imports
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// WPILib imports
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Local imports
import frc.robot.Constants.Vision;
import frc.robot.util.RobotManager;
import frc.robot.util.models.AprilTagTracker;

public class VisionTrackingSubsystem extends SubsystemBase {

	// Fields
	private final PhotonCamera photonCamera;
	private PhotonPoseEstimator photonPoseEstimator;
	private final List<AprilTagTracker> aprilTagTrackers;

	// Constructor
	public VisionTrackingSubsystem() {
		photonCamera = new PhotonCamera(Vision.CAMERA_NAME);
		aprilTagTrackers = initializeAprilTagTrackers();
		initializePhotonPoseEstimator();
	}

	// Initialization Methods
	private List<AprilTagTracker> initializeAprilTagTrackers() {
		List<AprilTagTracker> trackers = new ArrayList<>();
		trackers.add(new AprilTagTracker("1", "6"));
		trackers.add(new AprilTagTracker("2", "7"));
		trackers.add(new AprilTagTracker("3", "8"));
		return trackers;
	}

	private void initializePhotonPoseEstimator() {
		try {
			AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
			photonPoseEstimator = new PhotonPoseEstimator(
					fieldLayout,
					PoseStrategy.MULTI_TAG_PNP,
					photonCamera,
					Vision.ROBOT_TO_CAM);
			photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			photonPoseEstimator = null;
		}
	}

	// Core Methods
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		if (photonPoseEstimator == null) {
			return Optional.empty();
		}
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update();
	}

	// Periodic Update
	@Override
	public void periodic() {
		// TODO: Replace with photonvision detected tags
		Set<String> detectedTags = new HashSet<>();
		detectedTags.add("1");
		detectedTags.add("7");

		// Update tag detection statuses based on the set of detected tags.
		updateTagDetection(detectedTags);

		updateSmartDashboard();
	}

	// Utility Methods
	private void updateSmartDashboard() {
		for (int i = 0; i < aprilTagTrackers.size(); ++i) {
			AprilTagTracker tracker = aprilTagTrackers.get(i);
			SmartDashboard.putBoolean("AprilTag " + (i + 1), tracker.isDetected);
		}
	}

	private void updateTagDetection(Set<String> tagIDs) {
		for (AprilTagTracker tracker : aprilTagTrackers) {
			String relevantTagId = getRelevantTagIdForAlliance(tracker);
			tracker.isDetected = tagIDs.contains(relevantTagId);
		}
	}

	private String getRelevantTagIdForAlliance(AprilTagTracker tracker) {
		switch (RobotManager.getAlliance()) {
			case Red:
				return tracker.redAprilTagId;
			case Blue:
				return tracker.blueAprilTagId;
			default:
				return null;
		}
	}
}
