package frc.robot.drivetrain;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Drivetrain extends SubsystemBase{

    /*
        Convention for motors, ALWAYS IN ORDER:
            Front;
            Back;
            Left;
            Right;
    */

    // Initiliaze member variables:

    // Modules (Wheels) on the robot
    private Module m_FrontModule;
    private Module m_BackModule;
    private Module m_LeftModule;
    private Module m_RightModule;

    // The direction the robot is pointing in, relative to field X [CCW+, In radians]
    private double m_fieldCentricBearing = 0;

    // Instance of the class, used so that only one instance of the class is being passed around
    private static Drivetrain m_instance;
    
    private Drivetrain(){
        CANSparkMax frontMotor = new CANSparkMax(DriveConstants.FRONT_MOTOR_ID, MotorType.kBrushed);
        CANSparkMax backMotor = new CANSparkMax(DriveConstants.BACK_MOTOR_ID, MotorType.kBrushed);
        CANSparkMax leftMotor = new CANSparkMax(DriveConstants.LEFT_MOTOR_ID, MotorType.kBrushed);
        CANSparkMax rightMotor = new CANSparkMax(DriveConstants.RIGHT_MOTOR_ID, MotorType.kBrushed);

        initMotors(frontMotor, backMotor, leftMotor, rightMotor);

        // Initiliazes every module with their respective (x, y) coordinate relative to origin and a motr
        m_FrontModule = new Module(new Vector2d(0, DriveConstants.TRACK_WIDTH_METERS/2), frontMotor);
        m_BackModule = new Module(new Vector2d(0, -DriveConstants.TRACK_WIDTH_METERS/2), backMotor);
        m_LeftModule = new Module(new Vector2d(-DriveConstants.TRACK_WIDTH_METERS/2, 0), leftMotor);
        m_RightModule = new Module(new Vector2d(DriveConstants.TRACK_WIDTH_METERS/2, 0), rightMotor);
    }

    /**
     * All code relating to the initilization of motors goes here:
     */
    private void initMotors(CANSparkMax ... motors){
        // Sets the inversion types of the motor, done so that the math doesn't have to account for negatives
        motors[0].setInverted(false);
        motors[1].setInverted(true);
        motors[2].setInverted(true);
        motors[3].setInverted(false);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    /**
     * Sets motor speeds based off of linear and angular acceleration. 
     * @param xSupplier: Target robot X velocity
     * @param ySupplier: Target robot Y velocity
     * @param thetaSupplier: Target robot angular velocity
     */
    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        // The target velocity magnitude
        double targetVel = Math.sqrt(Math.pow(xSupplier.getAsDouble(), 2) + Math.pow(ySupplier.getAsDouble(), 2));
        // The target velocity direction relative to field X [CCW+, In radians]
        double fieldCentricHeading = Math.asin(ySupplier.getAsDouble() / targetVel);
        // The target velocity deirection relative to robot front [CCW+, In radians]
        double robotCentricBearing = fieldCentricHeading - m_fieldCentricBearing;

        // TODO add comment explaining this
        double rotation = (thetaSupplier.getAsDouble() * DriveConstants.TRACK_WIDTH_METERS / 2) / ModuleConstants.WHEEL_RADIUS;
        // TODO add comment explaning this
        double translationCOS = (targetVel * Math.cos(robotCentricBearing)) / ModuleConstants.WHEEL_RADIUS;
        double translationSIN = (targetVel * Math.sin(robotCentricBearing)) / ModuleConstants.WHEEL_RADIUS;

        m_FrontModule.setRPM((translationSIN + rotation) * Constants.RADPS_TO_RPM);
        m_BackModule.setRPM((translationSIN - rotation) * Constants.RADPS_TO_RPM);
        m_LeftModule.setRPM((translationCOS - rotation) * Constants.RADPS_TO_RPM);
        m_RightModule.setRPM((translationCOS + rotation) * Constants.RADPS_TO_RPM);
    }

    /**
     * @return returns the current instance of the drivetrain
     */
    public static Drivetrain getInstance(){
        return m_instance = m_instance == null ? new Drivetrain() : m_instance;
    }

}
