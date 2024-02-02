package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax m_leftPivotMotor;
    private CANSparkMax m_rightPivotMotor;

    private double m_pivotSpeed;

    private GenericEntry pivotSpeedEntry;
    
    public PivotSubsystem() {
        m_leftPivotMotor = new CANSparkMax (Constants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax (Constants.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.setIdleMode(IdleMode.kBrake);

        ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

        pivotSpeedEntry = pivotTab.add("Pivot Speed", 0).getEntry();
    }

    /* Sets speed of the shooter based on axis values of Joystick. */
    public void pivotRotate(double pivotSpeed) {
        m_pivotSpeed = pivotSpeed;
    }

    @Override
    public void periodic() {
        m_leftPivotMotor.set(m_pivotSpeed);
        m_rightPivotMotor.set(-m_pivotSpeed);    }
}
