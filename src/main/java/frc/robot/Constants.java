package frc.robot;

public class Constants {

    public static int DRIVE_CONTROLLER_PORT = 1;

    public static double RADPS_TO_RPM = 30d / Math.PI;

    public static class ModuleConstants{
        /**
         * Radius of the wheel [in meters]
         */
        public static double WHEEL_RADIUS = 0.1;
    }

    public static class DriveConstants{
        /**
         * Length of one side of the robot [in meters]
         */
        public static double TRACK_WIDTH_METERS = 10;

        // Motor IDs:
        public static int FRONT_MOTOR_ID = 1;
        public static int BACK_MOTOR_ID = 2;
        public static int LEFT_MOTOR_ID = 3;
        public static int RIGHT_MOTOR_ID = 4;
    }
}
