package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsystem;

public class PositionIntakeCommand extends Command {
    private PivotSubsystem m_pivotSubsystem;
    private PIDController m_pivotPIDController;

    private String m_pivotPosition;

    // Desired position setpoints
    private static final double ZERO_POSITION_RAD = 0; // Assuming 0 radians is the zero position
    private static final double MID_POSITION_RAD = Math.PI / 4; // Mid position at 45 degrees
    private static final double TOP_POSITION_RAD = Math.PI / 2; // Top position at 90 degrees

    public PositionIntakeCommand(PivotSubsystem pivotSubsystem, String pivotPosition) {
        m_pivotSubsystem = pivotSubsystem;

        m_pivotPosition = pivotPosition;

        m_pivotPIDController = new PIDController(0.005, 0, 0);
        addRequirements(m_pivotSubsystem);
    }

    @Override
    public void execute() {
        double setpoint = getDesiredPosition();
        double currentPos = m_pivotSubsystem.getCurrentPivotPosition();
        double pidOutput = m_pivotPIDController.calculate(setpoint - currentPos);

        m_pivotSubsystem.pivotRotate(pidOutput);
    }

    private double getDesiredPosition() {
        if (m_pivotPosition == "Zero") {
            return 13;
        } else if (m_pivotPosition == "Mid") {
            return 32;
        } else if (m_pivotPosition == "Top") {
            return 70;
        }
        return 0; // Default to zero position if no button is pressed
    }

    @Override
    public void end(boolean interrupted) {
        m_pivotSubsystem.pivotRotate(0); // Stop the pivot when the command ends
    }
}
