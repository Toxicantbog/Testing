package frc.robot;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Get this distance in meters, convert it.
    //public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;/// distance from left wheels to right wheels
    //public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;// distance from front motors to back motors

    // Change the decimal numbers here to inches. If the distance between the wheels is 40 inches,
    // then type a 40 in place of the '0.2794'. If the distance is 40 and a half inches, type it
    // in as 40.5.
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24);

    public static final int DRIVETRAIN_PIGEON_ID = 0; // don't need this, but if we get one, we need to assign it an ID

    //Change the numbers to match the ID's, for the drive, steer, and encoder
    // Set the Encoder ID in the Phoenix Tuner software
    // Mess with the offset values so that the robot wheels end up straight, they are degree values
    // The stock offset value is 0, if we ever want to return to "default" settings
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-62); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(18);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(237);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(217);
}
