package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command{
    private IntakeSubsystem m_intakeSubsystem;

    private DoubleSupplier m_intakeShooterVelocity;

    private double m_shooterVelocity;
    
    private BooleanSupplier m_turboIntakeNote;
    private DoubleSupplier m_intakeNote;
    private BooleanSupplier m_outtakeNote;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeShooterVelocity, DoubleSupplier intakeNote, BooleanSupplier outtakeNote, BooleanSupplier turboIntakeNote) {
        m_intakeSubsystem = intakeSubsystem;
        m_intakeShooterVelocity = intakeShooterVelocity;
        m_intakeNote = intakeNote;
        m_outtakeNote = outtakeNote;
        m_turboIntakeNote = turboIntakeNote;

        addRequirements(m_intakeSubsystem);
    }

    public void execute() {
        if (Math.abs(m_intakeNote.getAsDouble()) > 0.01) { m_intakeSubsystem.intakeRotate(-0.8); }
        else if (m_outtakeNote.getAsBoolean()) { m_intakeSubsystem.intakeRotate(0.8); } 
        else if (m_turboIntakeNote.getAsBoolean()) { m_intakeSubsystem.intakeRotate(-1); }
        else {
            m_intakeSubsystem.intakeRotate(0);
        }

        m_shooterVelocity = m_intakeShooterVelocity.getAsDouble();
        m_intakeSubsystem.shooterRotate(m_shooterVelocity);
    }

    @Override
    public void end(boolean interrupted) { m_intakeSubsystem.intakeRotate(0); m_intakeSubsystem.shooterRotate(0);}
}
