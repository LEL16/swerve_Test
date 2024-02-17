package frc.robot.Subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LinearActuatorSubsystem extends SubsystemBase {
    private final CANSparkMax m_actuatorMotor;
    private final SparkAbsoluteEncoder m_actuatorEncoder;

    private final GenericEntry m_actuatorPositionEntry, m_acutatorSpeedEntry;

    public LinearActuatorSubsystem(){
        m_actuatorMotor = new CANSparkMax(Constants.LINEAR_ACTUATOR_MOTOR, MotorType.kBrushed);
        

        m_actuatorMotor.setIdleMode(IdleMode.kBrake);
        m_actuatorEncoder = m_actuatorMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout actuatorLayout = tab.getLayout("Actuator", BuiltInLayouts.kList).withSize(2,6).withPosition(0,0);

        m_actuatorPositionEntry = actuatorLayout.add("Encoder Position", m_actuatorEncoder.getPosition()).getEntry();
        m_acutatorSpeedEntry = actuatorLayout.add("Speed", m_actuatorEncoder.getVelocity()).getEntry();
    }

    public void updateShuffleboard() {
        m_actuatorPositionEntry.setDouble(m_actuatorEncoder.getPosition());
        m_acutatorSpeedEntry.setDouble(m_actuatorEncoder.getVelocity());
    }

    @Override
    public void periodic(){
        updateShuffleboard();
    }

    public void actuate(double speed) {
        m_actuatorMotor.set(speed);
    }
}
