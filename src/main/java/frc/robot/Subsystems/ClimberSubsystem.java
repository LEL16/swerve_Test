package frc.robot.Subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.DefaultClimberCommand;


public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    public ClimberSubsystem(){
        m_leftMotor = new CANSparkMax(Constants.LEFT_CLIMBER, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(Constants.RIGHT_CLIMBER, MotorType.kBrushless);

        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setIdleMode(IdleMode.kBrake);

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    }

    @Override
    public void periodic(){
        
    }

    public void rotate(double Speed) {
        m_rightMotor.set(3);
        m_leftMotor.set(-.3);
    }
}
