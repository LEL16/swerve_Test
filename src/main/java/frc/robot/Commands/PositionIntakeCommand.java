package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsystem;

public class PositionIntakeCommand extends Command{
    private PivotSubsystem m_pivotSubsystem;

    private double m_pivotPositionRad;

    private PIDController m_pivotPIDController;

    private BooleanSupplier m_zeroPosition;
    private BooleanSupplier m_midPosition;
    private BooleanSupplier m_topPosition;

    PositionIntakeCommand(PivotSubsystem pivotSubsystem, double pivotPositionRad, BooleanSupplier zeroPosition, BooleanSupplier midPosition, BooleanSupplier topPosition) {
        m_pivotSubsystem = pivotSubsystem;
        m_pivotPositionRad = m_pivotPositionRad;

        m_zeroPosition = zeroPosition;
        m_midPosition = midPosition;
        m_topPosition = topPosition; 

        m_pivotPIDController = new PIDController(0.5, 0.02, 0.01);

        addRequirements(m_pivotSubsystem);
    }

    public void execute() {
       // pivotPositionRad = m_
       // m_pivotPIDController(m);
    }

    @Override
    public void end(boolean interrupted) { m_pivotSubsystem.pivotRotate(0); }
}
