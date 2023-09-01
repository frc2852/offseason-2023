// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public final class Constants {

	public static final int INTAKE_LEFT = 13;
	public static final int INTAKE_RIGHT = 14;
	public static final int INTAKE_TOP = 15;

	public static class VisionConstants {
		public static final Transform3d robotToCam = new Transform3d(
				new Translation3d(0.5, 0.0, 0.5),
				new Rotation3d(
						0, 0,
						0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
								// from center.
		public static final String cameraName = "YOUR CAMERA NAME";
	}
}
