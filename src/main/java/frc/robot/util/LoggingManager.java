package frc.robot.util;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class LoggingManager {
    public enum MessageType {
        DEBUG,
        WARNING,
        ERROR
    }

    static {
        initializeDataLog();
    }

    private static void initializeDataLog() {
        if (!RobotManager.isSimulation()) {
            DataLogManager.start();
            DriverStation.startDataLog(DataLogManager.getLog());
        }
    }

    public static void log(String message) {
        log(message, MessageType.DEBUG);
    }

    public static void log(String message, MessageType type) {
        String logMessage = formatLogMessage(message, type);
        writeToDriverStation(logMessage, type);
        writeToDataLog(logMessage);
    }

    private static String formatLogMessage(String message, MessageType type) {
        return String.format("[%s] %s", type.toString(), message);
    }

    private static void writeToDriverStation(String logMessage, MessageType type) {
        if (RobotManager.isSimulation() && type == MessageType.DEBUG) {
            DriverStation.reportWarning(logMessage, false);
        } else {
            switch (type) {
                case WARNING:
                    DriverStation.reportWarning(logMessage, false);
                    break;
                case ERROR:
                    DriverStation.reportError(logMessage, false);
                    break;
                default:
                    break;
            }
        }
    }

    private static void writeToDataLog(String logMessage) {
        DataLogManager.log(logMessage);
    }
}
