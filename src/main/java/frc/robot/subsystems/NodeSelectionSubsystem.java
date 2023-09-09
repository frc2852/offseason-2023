// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggingManager;
import frc.robot.util.RobotManager;
import frc.robot.util.LoggingManager.MessageType;
import frc.robot.util.models.Node;

public class NodeSelectionSubsystem extends SubsystemBase {

	// Fields
	private final List<Node> grid = new ArrayList<>();
	private int selectedIndex = 0;

	// Constructor
	public NodeSelectionSubsystem() {
		initializeGrid();

	}

	// Initialization Methods
	private void initializeGrid() {
		grid.add(new Node("Node 1", "1", "6", true));
		grid.add(new Node("Node 2", "1", "6", false));
		grid.add(new Node("Node 3", "1", "6", false));
		grid.add(new Node("Node 4", "2", "7", false));
		grid.add(new Node("Node 5", "2", "7", false));
		grid.add(new Node("Node 6", "2", "7", false));
		grid.add(new Node("Node 7", "3", "8", false));
		grid.add(new Node("Node 8", "3", "8", false));
		grid.add(new Node("Node 9", "3", "8", false));
	}

	// Core Functionality Methods
	public void cycleRight() {
		grid.get(selectedIndex).selected = false;
		selectedIndex = (selectedIndex - 1 + grid.size()) % grid.size();
		grid.get(selectedIndex).selected = true;
	}

	public void cycleLeft() {
		grid.get(selectedIndex).selected = false;
		selectedIndex = (selectedIndex + 1) % grid.size();
		grid.get(selectedIndex).selected = true;
	}

	// Getter Methods
	public Node getSelectedZone() {
		return grid.get(selectedIndex);
	}

	public String getSelectedAprilTagId() {
		Node zone = getSelectedZone();
		switch (RobotManager.getAlliance()) {
			case Red:
				return zone.redAprilTagId;
			case Blue:
				return zone.blueAprilTagId;
			default: {
				LoggingManager.log("Attempted to selected a Node without a valid alliance selected", MessageType.ERROR);
				return null;
			}
		}
	}

	@Override
	public void periodic() {
		updateDashboard();
	}

	private void updateDashboard() {
		for (int i = 0; i < grid.size(); i++) {
			Node node = grid.get(i);
			SmartDashboard.putBoolean(node.description, node.selected);
		}
	}
}
