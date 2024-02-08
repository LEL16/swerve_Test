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

    public PivotSubsystem() {
        m_leftPivotMotor = new CANSparkMax(Constants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax(Constants.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

        // Set motors to brake mode to hold position when not moving
        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.setIdleMode(IdleMode.kBrake);

        // Assuming the left pivot motor's encoder is used for feedback
        m_pivotEncoder = m_leftPivotMotor.getEncoder();

        // Set the position conversion factor based on your gear ratio
        m_pivotEncoder.setPositionConversionFactor(1); // Example value, adjust as needed

        // Shuffleboard setup for monitoring, not essential for functionality
        ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");
        pivotAngleEntry = pivotTab.add("Pivot Angle", 0).getEntry();
    }

    // Method to set pivot rotation speed
    public void pivotRotate(double speed) {
        m_leftPivotMotor.set(speed);
        // Inverting speed for the right motor, adjust if necessary for your design
        m_rightPivotMotor.set(-speed);
    }

    // Method to get current pivot position from the encoder
    public double getCurrentPivotPosition() {
        return m_pivotEncoder.getPosition();
    }

    // Ensure motors stop when not being controlled
    @Override
    public void periodic() {
        pivotAngleEntry.setDouble(getCurrentPivotPosition());
    }
}
