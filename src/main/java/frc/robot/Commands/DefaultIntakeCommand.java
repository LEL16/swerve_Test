// package frc.robot.Commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.IntakeSubsystem;

// public class DefaultIntakeCommand extends Command{
//     private IntakeSubsystem m_intakeSubsystem;

//     private DoubleSupplier m_shooterVelocity;
    
//     private BooleanSupplier m_turboIntakeNote;
//     private BooleanSupplier m_intakeNote;
//     private BooleanSupplier m_outtakeNote;

//     public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier shooterVelocity, BooleanSupplier intakeNote, BooleanSupplier outtakeNote, BooleanSupplier turboIntakeNote) {
//         m_intakeSubsystem = intakeSubsystem;
//         m_shooterVelocity = shooterVelocity;
//         m_intakeNote = intakeNote;
//         m_outtakeNote = outtakeNote;
//         m_turboIntakeNote = turboIntakeNote;

//         addRequirements(m_intakeSubsystem);
//     }

//     public void execute() {
//         if (m_intakeNote.getAsBoolean()) { m_intakeSubsystem.intakeRotate(-0.25); }
//         else if (m_outtakeNote.getAsBoolean()) { m_intakeSubsystem.intakeRotate(0.25); } 
//         else if (m_turboIntakeNote.getAsBoolean()) { m_intakeSubsystem.intakeRotate(-1); }

//         m_intakeSubsystem.shooterRotate(m_shooterVelocity.getAsDouble());
//     }

//     @Override
//     public void end(boolean interrupted) { m_intakeSubsystem.intakeRotate(0); m_intakeSubsystem.shooterRotate(0);}
// }
