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
}
