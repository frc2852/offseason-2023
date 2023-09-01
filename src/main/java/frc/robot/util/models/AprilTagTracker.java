package frc.robot.util.models;

public class AprilTagTracker {
	public final String redAprilTagId;
	public final String blueAprilTagId;
	public boolean isDetected;

	public AprilTagTracker(String redAprilTagId, String blueAprilTagId) {
		this.redAprilTagId = redAprilTagId;
		this.blueAprilTagId = blueAprilTagId;
		this.isDetected = false;
	}

	// Check if the given set of tagIDs matches either red or blue AprilTag IDs
	public boolean matchesId(String tagID) {
		return redAprilTagId.equals(tagID) || blueAprilTagId.equals(tagID);
	}
}
