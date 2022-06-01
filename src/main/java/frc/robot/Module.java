package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase{

    Vector2d m_wheelPos;
    CANSparkMax m_motor;
    SparkMaxPIDController m_PidController;
    
    
    /**
     * 
     * @param position: Position of the wheel relative to the robot center
     * @param motor: Motor the wheel will be controlled with
     */
    public Module(Vector2d position, CANSparkMax motor){
        m_wheelPos = position;
        m_motor = motor;
        m_PidController = m_motor.getPIDController();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void setRPM(double rpm){
        m_PidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

}
