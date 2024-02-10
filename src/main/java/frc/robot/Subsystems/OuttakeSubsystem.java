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

public class OuttakeSubsystem extends SubsystemBase {
    private CANSparkMax m_leftShooterMotor;
    private CANSparkMax m_rightShooterMotor;

    private double m_shooterSpeed;

    private GenericEntry shooterSpeedEntry;

    public OuttakeSubsystem() {
        m_leftShooterMotor = new CANSparkMax(Constants.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(Constants.RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);

        m_leftShooterMotor.setIdleMode(IdleMode.kBrake);
        m_rightShooterMotor.setIdleMode(IdleMode.kBrake);

        ShuffleboardTab outtakeTab = Shuffleboard.getTab("Outtake");

        shooterSpeedEntry = outtakeTab.add("Outtake Speed", 0).getEntry();
    }

    /**
     * Rotates the shooter motors at the specified speed.
     * 
     * @param speed The speed at which to rotate the shooter motors. Positive values
     *              rotate clockwise, negative values rotate counterclockwise.
     */
    public void outtakeRotate(double speed) {
        m_shooterSpeed = -speed * 1.25;
        m_leftShooterMotor.set(m_shooterSpeed);
        m_rightShooterMotor.set(m_shooterSpeed);
    }

    @Override
    public void periodic() {
        // Update the shooter speed entry on Shuffleboard with the current speed of the
        // shooter motors
        shooterSpeedEntry.setDouble(m_shooterSpeed);
    }
}
