package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    private final DoubleSupplier m_intakeRateSupplier;
    private final DoubleSupplier m_angularSpeedSupplier;
    private final BooleanSupplier m_reverseButtonSupplier;

    private final double REVERSE_SPEED = 0.4 * IntakeSubsystem.kIntakeMaxRate * 0.5;

    /**
     * Command to engage the intake using joystick input.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param intakeRateSupplier The desired intake rate (rpm).
     * @param angularSpeedSupplier The desired angle change speed (rad/s).
     */
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeRateSupplier, DoubleSupplier angularSpeedSupplier, BooleanSupplier reverseButtonSupplier) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_intakeRateSupplier = intakeRateSupplier;
        this.m_angularSpeedSupplier = angularSpeedSupplier;
        this.m_reverseButtonSupplier = reverseButtonSupplier;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.intake(m_reverseButtonSupplier.getAsBoolean() ? REVERSE_SPEED : m_intakeRateSupplier.getAsDouble());
        m_intakeSubsystem.rotate(m_angularSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_intakeSubsystem.intake(0);
        m_intakeSubsystem.rotate(0); 
    }
}