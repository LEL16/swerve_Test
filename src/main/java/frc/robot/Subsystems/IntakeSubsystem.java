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
    private CANSparkMax m_leftShooterMotor;
    private CANSparkMax m_rightShooterMotor;

    private CANSparkMax m_leftIntakeMotor;
    private CANSparkMax m_rightIntakeMotor;

    
    private RelativeEncoder m_leftRelativeEncoder;
    private RelativeEncoder m_rightRelativeEncoder;

    private double m_shooterSpeed;
    private double m_intakeSpeed;

    private GenericEntry shooterSpeedEntry;
    private GenericEntry flywheelSpeedEntry;
    
    public IntakeSubsystem() {
        m_leftShooterMotor = new CANSparkMax (Constants.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax (Constants.RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
        m_leftIntakeMotor = new CANSparkMax (Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        m_rightIntakeMotor = new CANSparkMax (Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

        m_leftShooterMotor.setIdleMode(IdleMode.kBrake);
        m_rightShooterMotor.setIdleMode(IdleMode.kBrake);
        m_leftIntakeMotor.setIdleMode(IdleMode.kBrake);
        m_rightIntakeMotor.setIdleMode(IdleMode.kBrake);

        m_leftRelativeEncoder = m_leftShooterMotor.getEncoder();
        m_rightRelativeEncoder = m_rightShooterMotor.getEncoder();

        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

        shooterSpeedEntry = intakeTab.add("Shooter Speed", 0).getEntry();
        flywheelSpeedEntry = Shuffleboard.getTab("Intake").add("Flywheel Speed", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    }

    /* Sets speed of the shooter based on axis values of Joystick. */
    public void shooterRotate(double shooterSpeed) {
        m_shooterSpeed = shooterSpeed;
    }

    /* Sets speed of the intake based on boolean values of Joystick triggers */
    public void intakeRotate(double intakeSpeed) {
        m_intakeSpeed = intakeSpeed;
    }

    @Override
    public void periodic() {
        m_leftIntakeMotor.set(m_intakeSpeed);
        m_rightIntakeMotor.set(m_intakeSpeed);
        
        m_leftShooterMotor.set(m_shooterSpeed);
        m_rightShooterMotor.set(m_shooterSpeed);

        shooterSpeedEntry.setDouble(m_shooterSpeed);
    }
}
