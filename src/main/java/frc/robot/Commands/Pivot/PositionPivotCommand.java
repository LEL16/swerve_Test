package frc.robot.Commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsystem;

public class PositionPivotCommand extends Command {
    private final PivotSubsystem m_pivotSubsystem;
    private final String m_position;

    private PIDController m_pivotPIDController;

    private GenericEntry PIDOutputEntry;
    private GenericEntry PIDSetpointEntry;
    private GenericEntry PIDErrorEntry;

    public PositionPivotCommand(PivotSubsystem pivotSubsystem, String position) {
        m_pivotSubsystem = pivotSubsystem;
        m_position = position;

        m_pivotPIDController = new PIDController(0.1, 0.1, 0.1);

        ShuffleboardLayout pivotShuffleboardLayout = Shuffleboard.getTab("Pivot")
                .getLayout("Pivot Shuffleboard Data", BuiltInLayouts.kList).withSize(2, 2);

        PIDOutputEntry = pivotShuffleboardLayout.add("PID Output " + m_position, 0).getEntry();
        PIDSetpointEntry = pivotShuffleboardLayout.add("PID Setpoint " + m_position, 0).getEntry();
        PIDErrorEntry = pivotShuffleboardLayout.add("PID Error " + m_position, 0).getEntry();

        addRequirements(m_pivotSubsystem);
    }

    @Override
    public void execute() {
        double PIDOutput = m_pivotPIDController.calculate(m_pivotSubsystem.getPivotAngle(), getDesiredPosition());

        PIDOutputEntry.setDouble(PIDOutput);
        PIDSetpointEntry.setDouble(getDesiredPosition());
        PIDErrorEntry.setDouble(m_pivotPIDController.getPositionError());

        m_pivotSubsystem.pivotRotate(PIDOutput);
    }

    @Override
    public void end(boolean interrupted) {
        m_pivotSubsystem.pivotRotate(0);
    }

    public double getDesiredPosition() {
        switch (m_position) {
            case "low":
                return 20;
            case "mid":
                return 40;
            case "high":
                return 60;
            default:
                return 40;
        }
    }
}
