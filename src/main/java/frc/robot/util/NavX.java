package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX {

    private final AHRS navX;

    // Constructor initializes the AHRS (gyro sensor)
    public NavX() {
      navX = new AHRS(SerialPort.Port.kMXP);
      zeroYaw();
    }

    /**
     * Zeroes the Yaw channel, which is the robot's heading
     */
    public void zeroYaw() {
        navX.zeroYaw();
    }

    /**
     * Returns the current rotation of the robot as a Rotation2d object
     * 
     * @return Current rotation of the robot
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    /**
     * Gets the heading in degrees from the gyro sensor
     * 
     * @return Heading in degrees
     */
    public double getHeadingDegrees() {
        try {
            return Math.IEEEremainder(-navX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
    }

    /**
     * Gets the turn rate in degrees per second
     * 
     * @return Turn rate in degrees per second
     */
    public double getTurnRate() {
        return navX.getRate();
    }
}
