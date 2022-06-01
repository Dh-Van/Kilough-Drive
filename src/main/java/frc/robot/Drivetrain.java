package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{

    /*
        Convention for naming:
            Front;
            Back;
            Left;
            Right;
    */

    Module m_FrontModule;
    Module m_BackModule;
    Module m_LeftModule;
    Module m_RightModule;

    CANSparkMax m_FrontMotor;
    CANSparkMax m_BackMotor;
    CANSparkMax m_LeftMotor;
    CANSparkMax m_RightMotor;
    
    public Drivetrain(){
        m_FrontMotor = new CANSparkMax(1, MotorType.kBrushed);
        m_BackMotor = new CANSparkMax(2, MotorType.kBrushed);
        m_LeftMotor = new CANSparkMax(3, MotorType.kBrushed);
        m_RightMotor = new CANSparkMax(4, MotorType.kBrushed);

        m_FrontModule = new Module(new Vector2d(1, 1), m_FrontMotor);
        m_BackModule = new Module(new Vector2d(1, 1), m_BackMotor);
        m_LeftModule = new Module(new Vector2d(1, 1), m_LeftMotor);
        m_RightModule = new Module(new Vector2d(1, 1), m_RightMotor);

    }

}
