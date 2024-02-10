package frc.robot.Commands.Pivot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsystem;

public class DefaultPivotCommand extends Command {
    private PivotSubsystem m_pivotSubsystem;
    private DoubleSupplier m_pivotVelocitySupplier;
    private BooleanSupplier m_pivotResetEncoders;

    /**
     * Constructs a new DefaultPivotCommand.
     * 
     * @param pivotSubsystem        The pivot subsystem to control.
     * @param pivotVelocitySupplier A supplier of the pivot velocity.
     * @param pivotResetEncoders    A supplier that provides the reset encoders
     *                              value.
     */
    public DefaultPivotCommand(PivotSubsystem pivotSubsystem, DoubleSupplier pivotVelocitySupplier,
            BooleanSupplier pivotResetEncoders) {
        m_pivotSubsystem = pivotSubsystem;
        m_pivotVelocitySupplier = pivotVelocitySupplier;
        m_pivotResetEncoders = pivotResetEncoders;

        addRequirements(m_pivotSubsystem);
    }

    public void execute() {
        if (m_pivotResetEncoders.getAsBoolean()) {
            m_pivotSubsystem.resetEncoders();
        }
        m_pivotSubsystem.pivotRotate(m_pivotVelocitySupplier.getAsDouble() / 3);
    }

    /**
     * Ends the default pivot behavior by stopping the pivot rotation.
     * 
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        m_pivotSubsystem.pivotRotate(0);
    }
}
