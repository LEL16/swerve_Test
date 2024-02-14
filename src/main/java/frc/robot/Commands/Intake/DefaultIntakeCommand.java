package frc.robot.Commands.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command {
    private IntakeSubsystem m_intakeSubsystem;

    private BooleanSupplier m_intakeNoteSupplier;
    private BooleanSupplier m_outtakeNoteSupplier;

    /**
     * Constructs a new DefaultIntakeCommand.
     * 
     * @param intakeSubsystem     The intake subsystem to control.
     * @param intakeNoteSupplier  A supplier that provides the intake note value.
     * @param outtakeNoteSupplier A supplier that provides the outtake note value.
     */
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier intakeNoteSupplier,
            BooleanSupplier outtakeNoteSupplier) {
        m_intakeSubsystem = intakeSubsystem;
        m_intakeNoteSupplier = intakeNoteSupplier;
        m_outtakeNoteSupplier = outtakeNoteSupplier;

        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute() {
        if (m_intakeNoteSupplier.getAsBoolean()) {
            m_intakeSubsystem.intakeRotate(-8);
        } else if (m_outtakeNoteSupplier.getAsBoolean()) {
            m_intakeSubsystem.intakeRotate(3);
        } else {
            m_intakeSubsystem.intakeRotate(0);
        }
    }

    /**
     * Ends the default intake behavior by stopping the intake rotation.
     * 
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intakeRotate(0);
    }
}
