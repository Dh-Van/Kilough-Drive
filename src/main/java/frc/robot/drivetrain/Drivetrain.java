package frc.robot.drivetrain;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.Vector2d;

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

    /**
     * Sets motor speeds based off of linear and angular acceleration. 
     * @param xSupplier: Linear X acceleration
     * @param ySupplier: Linear Y acceleration
     * @param thetaSupplier: Angular acceleration
     */
    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        double rotation = (thetaSupplier.getAsDouble() * DriveConstants.TRACK_WIDTH_METERS / 2) / ModuleConstants.WHEEL_RADIUS;
        double translation = (xSupplier.getAsDouble() / ModuleConstants.WHEEL_RADIUS);
        double heading = Math.tan(xSupplier.getAsDouble() / ySupplier.getAsDouble());

        double targetVel = rotation + translation;
        double targetRPM = targetVel / (60 * Math.PI);

        m_FrontModule.setRPM(targetRPM * Math.cos(heading));
        m_BackModule.setRPM(targetRPM * Math.cos(heading));
        m_LeftModule.setRPM(targetRPM * Math.sin(heading));
        m_RightModule.setRPM(targetRPM * Math.sin(heading));
    }

    /**
     * @return returns the current instance of the drivetrain
     */
    public static Drivetrain getInstance(){
        return m_instance = m_instance == null ? new Drivetrain() : m_instance;
    }

}
