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

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_intakeMotor;

    private double m_intakeSpeed;

    private GenericEntry intakeSpeedEntry;

    public IntakeSubsystem() {
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        intakeSpeedEntry = intakeTab.add("Intake Speed", 0).getEntry();
    }

    /**
     * Rotates the intake with the specified speed.
     * 
     * @param intakeSpeed the speed at which the intake should rotate
     */
    public void intakeRotate(double intakeSpeed) {
        m_intakeSpeed = -intakeSpeed;
        m_intakeMotor.set(m_intakeSpeed);
    }

    @Override
    public void periodic() {
        // Update the intake speed entry on Shuffleboard with the current speed of the
        // intake motor
        intakeSpeedEntry.setDouble(m_intakeSpeed);
    }
}
