package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsystem;

public class PositionPivotCommand extends Command {
    private PivotSubsystem m_pivotSubsystem;
    private PIDController m_pivotPIDController;

    private String m_pivotPosition;

    private GenericEntry pidOutputEntry;

    // Desired position setpoints
    private static final double ZERO_POSITION_RAD = 0; // Assuming 0 radians is the zero position
    private static final double MID_POSITION_RAD = Math.PI / 4; // Mid position at 45 degrees
    private static final double TOP_POSITION_RAD = Math.PI / 2; // Top position at 90 degrees

    public PositionPivotCommand(PivotSubsystem pivotSubsystem, String pivotPosition) {
        m_pivotSubsystem = pivotSubsystem;

        m_pivotPosition = pivotPosition;

        // ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");
        // pidOutputEntry = pivotTab.add("Pivot PID Output", 0).getEntry();

        m_pivotPIDController = new PIDController(0.5, 0.02, 0.01);
        addRequirements(m_pivotSubsystem);
    }

    @Override
    public void execute() {
        double setpoint = getDesiredPosition();
        double currentPos = m_pivotSubsystem.getCurrentPivotPosition();
        double pidOutput = m_pivotPIDController.calculate(currentPos, setpoint);

        // pidOutputEntry.setDouble(pidOutput);

        m_pivotSubsystem.pivotRotate(pidOutput);
    }

    private double getDesiredPosition() {
        if (m_pivotPosition == "Zero") {
            return ZERO_POSITION_RAD;
        } else if (m_pivotPosition == "Mid") {
            return MID_POSITION_RAD;
        } else if (m_pivotPosition == "Top") {
            return TOP_POSITION_RAD;
        }
        return 0; // Default to zero position if no button is pressed
    }

    @Override
    public void end(boolean interrupted) {
        m_pivotSubsystem.pivotRotate(0); // Stop the pivot when the command ends
    }
}
