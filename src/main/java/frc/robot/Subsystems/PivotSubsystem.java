package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax m_leftPivotMotor;
    private CANSparkMax m_rightPivotMotor;

    private RelativeEncoder m_pivotEncoder;

    private GenericEntry pivotAngleEntry;
    private GenericEntry pivotSpeedEntry;

    public PivotSubsystem() {
        m_leftPivotMotor = new CANSparkMax(Constants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax(Constants.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);
        m_leftPivotMotor.restoreFactoryDefaults();
        m_rightPivotMotor.restoreFactoryDefaults();
        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.setIdleMode(IdleMode.kBrake);
        m_leftPivotMotor.setSmartCurrentLimit(10);
        m_rightPivotMotor.setSmartCurrentLimit(10);

        m_pivotEncoder = m_leftPivotMotor.getEncoder();
        m_pivotEncoder.setPositionConversionFactor(1 / 3.4 * 360); // RPM to degrees
        m_pivotEncoder.setVelocityConversionFactor(1 / 3.4 * 360 / 60);

        ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");
        pivotAngleEntry = pivotTab.add("Pivot Angle", m_pivotEncoder.getPosition()).getEntry(); // Check default value
        pivotSpeedEntry = pivotTab.add("Pivot Speed", m_pivotEncoder.getVelocity()).getEntry();
    }

    @Override
    public void periodic() {
        // Update the pivot angle entry on Shuffleboard with the current position of the
        // pivot encoder
        pivotAngleEntry.setDouble(m_pivotEncoder.getPosition());
        // Update the pivot speed entry on Shuffleboard with the current speed of the
        // pivot motors
        pivotSpeedEntry.setDouble(m_pivotEncoder.getVelocity());
    }

    /**
     * Rotate the pivot motors at the specified speed.
     * 
     * @param speed The speed at which to rotate the pivot motors. Positive values
     *              rotate clockwise, negative values rotate counterclockwise.
     */
    public void pivotRotate(double speed) {
        m_leftPivotMotor.set(speed);
        m_rightPivotMotor.set(-speed);
    }

    /**
     * Returns the current angle of the pivot.
     * 
     * @return The current angle of the pivot.
     */
    public double getPivotAngle() {
        return m_pivotEncoder.getPosition();
    }

    /**
     * Resets the pivot encoder to 0.
     */
    public void resetEncoders() {
        m_pivotEncoder.setPosition(0);
    }
}
