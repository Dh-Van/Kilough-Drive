package frc.robot.drivetrain;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase{

    /*
        Convention for naming:
            Front;
            Back;
            Left;
            Right;
    */

    private Module m_FrontModule;
    private Module m_BackModule;
    private Module m_LeftModule;
    private Module m_RightModule;

    private CANSparkMax m_FrontMotor;
    private CANSparkMax m_BackMotor;
    private CANSparkMax m_LeftMotor;
    private CANSparkMax m_RightMotor;

    private static Drivetrain m_instance;
    
    private Drivetrain(){
        m_FrontMotor = new CANSparkMax(DriveConstants.FRONT_MOTOR_ID, MotorType.kBrushed);
        m_BackMotor = new CANSparkMax(DriveConstants.BACK_MOTOR_ID, MotorType.kBrushed);
        m_LeftMotor = new CANSparkMax(DriveConstants.LEFT_MOTOR_ID, MotorType.kBrushed);
        m_RightMotor = new CANSparkMax(DriveConstants.RIGHT_MOTOR_ID, MotorType.kBrushed);

        initMotors();

        // Initiliazes every module with their respective (x, y) coordinate relative to origin and a motr
        m_FrontModule = new Module(new Vector2d(0, DriveConstants.TRACK_WIDTH_METERS/2), m_FrontMotor);
        m_BackModule = new Module(new Vector2d(0, -DriveConstants.TRACK_WIDTH_METERS/2), m_BackMotor);
        m_LeftModule = new Module(new Vector2d(-DriveConstants.TRACK_WIDTH_METERS/2, 0), m_LeftMotor);
        m_RightModule = new Module(new Vector2d(DriveConstants.TRACK_WIDTH_METERS/2, 0), m_RightMotor);
    }

    private void initMotors(){
        // Sets the inversion types of the motor, done so that the math doesn't have to account for negatives
        m_FrontMotor.setInverted(false);
        m_BackMotor.setInverted(true);
        m_LeftMotor.setInverted(true);
        m_RightMotor.setInverted(false);
    }

    /**
     * 
     * @param xSupplier: Linear X acceleration
     * @param ySupplier: Linear Y acceleration
     * @param thetaSupplier: Angular acceleration
     */
    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        double rotation = (thetaSupplier.getAsDouble() * DriveConstants.TRACK_WIDTH_METERS / 2) / Constants.WHEEL_RADIUS;
        double translation = (xSupplier.getAsDouble() / Constants.WHEEL_RADIUS);
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
