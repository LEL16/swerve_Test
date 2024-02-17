package frc.robot.Subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    public ClimberSubsystem(){
        m_leftMotor = new CANSparkMax(Constants.LEFT_CLIMBER, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(Constants.RIGHT_CLIMBER, MotorType.kBrushless);
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();
        m_leftMotor.setSmartCurrentLimit(20);
        m_rightMotor.setSmartCurrentLimit(20);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.follow(m_rightMotor, true);

        ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    }

    @Override
    public void periodic(){
        
    }

    public void climberRotate(double Speed) {
        m_rightMotor.setVoltage(5);
    }
}
