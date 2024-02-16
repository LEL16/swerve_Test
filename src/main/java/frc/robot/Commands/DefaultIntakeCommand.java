package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    private final BooleanSupplier m_outtakeRateSupplier;
    private final DoubleSupplier m_angularSpeedSupplier;
    private final DoubleSupplier m_intakeRateSupplier;

    /**
     * Command to engage the intake using joystick input.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param outtakeRateSupplier The desired intake rate (rpm).
     * @param angularSpeedSupplier The desired angle change speed (rad/s).
     */
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier outtakeRateSupplier, DoubleSupplier angularSpeedSupplier, DoubleSupplier intakeRateSupplier) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;
        this.m_angularSpeedSupplier = angularSpeedSupplier;
        this.m_intakeRateSupplier = intakeRateSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (Math.abs(m_intakeRateSupplier.getAsDouble()) > 0.05) {
            m_intakeSubsystem.intake(0.5);
        }
        else if (m_outtakeRateSupplier.getAsBoolean()) {
            m_intakeSubsystem.intake(-0.5);
        }
        m_intakeSubsystem.rotate(m_angularSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_intakeSubsystem.intake(0);
        m_intakeSubsystem.rotate(0); 
    }
}