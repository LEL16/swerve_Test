package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.PivotSubsystem;

public class DefaultPivotCommand extends Command{
    private PivotSubsystem m_pivotSubsystem;

    private DoubleSupplier m_pivotVelocity;

    public DefaultPivotCommand(PivotSubsystem pivotSubsystem, DoubleSupplier pivotVelocity) {
        m_pivotSubsystem = pivotSubsystem;
        m_pivotVelocity = pivotVelocity;

        addRequirements(m_pivotSubsystem);
    }

    public void execute() {
        m_pivotSubsystem.pivotRotate(m_pivotVelocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { m_pivotSubsystem.pivotRotate(0); }
}
