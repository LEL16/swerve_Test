package frc.robot.Commands.Outtake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class DefaultOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final DoubleSupplier m_outtakeVelocitySupplier;

    /**
     * Constructs a new DefaultOuttakeCommand.
     * 
     * @param outtakeSubsystem        The outtake subsystem to control.
     * @param outtakeVelocitySupplier A supplier of the outtake velocity.
     */
    public DefaultOuttakeCommand(OuttakeSubsystem outtakeSubsystem, DoubleSupplier outtakeVelocitySupplier) {
        m_outtakeSubsystem = outtakeSubsystem;
        m_outtakeVelocitySupplier = outtakeVelocitySupplier;

        addRequirements(m_outtakeSubsystem);
    }

    public void execute() {
        m_outtakeSubsystem.outtakeRotate(m_outtakeVelocitySupplier.getAsDouble());
    }

    /**
     * Ends the default outtake behavior by stopping the outtake rotation.
     * 
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        m_outtakeSubsystem.outtakeRotate(0);
    }

}
