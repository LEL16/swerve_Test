package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command{
    private IntakeSubsystem m_intakeSubsystem;

    private DoubleSupplier m_pivotVelocity;
    private BooleanSupplier m_intakeNote;
    private BooleanSupplier m_outtakeNote;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier pivotVelocity, BooleanSupplier intakeNote, BooleanSupplier outtakeNote) {
        m_intakeSubsystem = intakeSubsystem;
        m_pivotVelocity = pivotVelocity;
        m_intakeNote = intakeNote;
        m_outtakeNote = outtakeNote;

        addRequirements(m_intakeSubsystem);
    }

    public void execute() {
        m_intakeSubsystem.rotate(m_pivotVelocity.getAsDouble() * 7.5);

        if(m_intakeNote.getAsBoolean()) {
            m_intakeSubsystem.intake();
        } else {
            m_intakeSubsystem.stop();
        }

        if(m_outtakeNote.getAsBoolean()) {
            m_intakeSubsystem.outtake();
        } 
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.rotate(0);
    }
}
