package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command{
    private IntakeSubsystem m_intakeSubsystem;

    private DoubleSupplier m_shooterVelocity;
    
    private BooleanSupplier m_turboIntakeNote;
    private BooleanSupplier m_intakeNote;
    private BooleanSupplier m_outtakeNote;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier shooterVelocity, BooleanSupplier intakeNote, BooleanSupplier outtakeNote, BooleanSupplier turboIntakeNote) {
        m_intakeSubsystem = intakeSubsystem;
        m_shooterVelocity = shooterVelocity;
        m_intakeNote = intakeNote;
        m_outtakeNote = outtakeNote;
        m_turboIntakeNote = turboIntakeNote;

        addRequirements(m_intakeSubsystem);
    }

    public void execute() {
        m_intakeSubsystem.rotate(m_shooterVelocity.getAsDouble());

        if (m_intakeNote.getAsBoolean()) { m_intakeSubsystem.rotate(-0.25); }
        if (m_outtakeNote.getAsBoolean()) { m_intakeSubsystem.rotate(0.25); } 
        if (m_turboIntakeNote.getAsBoolean()) { m_intakeSubsystem.rotate(-1); }
    }

    @Override
    public void end(boolean interrupted) { m_intakeSubsystem.rotate(0); }
}
