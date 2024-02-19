package frc.robot.Commands.Outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final DoubleSupplier m_outtakeRateSupplier;
    private final DoubleSupplier m_linearActuatorRateSupplier;

    /**
     * Command to engage the outtake using joystick input.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param linearActuatorRateSupplier
     */
    public DefaultOuttakeCommand(OuttakeSubsystem outtakeSubsystem, DoubleSupplier outtakeRateSupplier, DoubleSupplier linearActuatorRateSupplier) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;
        this.m_linearActuatorRateSupplier = linearActuatorRateSupplier;

        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        m_outtakeSubsystem.outtake(m_outtakeRateSupplier.getAsDouble());
        if (Math.abs(m_linearActuatorRateSupplier.getAsDouble()) > 0.05) { m_outtakeSubsystem.actuate(-m_linearActuatorRateSupplier.getAsDouble() * 0.5); } else { m_outtakeSubsystem.actuate(0); }
        m_outtakeSubsystem.actuate(m_linearActuatorRateSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
        m_outtakeSubsystem.actuate(0);
    }
}