package frc.robot.util.models;

public class Node {
	public String description;
	public String redAprilTagId;
	public String blueAprilTagId;
	public boolean selected;

	public Node(String description, String redAprilTagId, String blueAprilTagId, boolean selected) {
		this.description = description;
		this.redAprilTagId = redAprilTagId;
		this.blueAprilTagId = blueAprilTagId;
		this.selected = selected;
	}

	@Override
	public String toString() {
		return "Node{" +
				"description='" + description + '\'' +
				", redAprilTagId='" + redAprilTagId + '\'' +
				", blueAprilTagId='" + blueAprilTagId + '\'' +
				", selected=" + selected +
				'}';
	}
}
