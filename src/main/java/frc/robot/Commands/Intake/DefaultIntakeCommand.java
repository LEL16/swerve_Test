package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    private final BooleanSupplier m_outtakeRateSupplier;
    private final DoubleSupplier m_pivotSpeedSupplier;
    private final DoubleSupplier m_intakeRateSupplier;

    /**
     * Command to engage the intake using joystick input.
     * 
     * @param intakeSubsystem     The intake subsystem.
     * @param pivotSpeedSupplier  The speed of the pivot (rad/s).
     * @param intakeRateSupplier  The rate of the intake (0 to 1).
     * @param outtakeRateSupplier The button for outtake (true or false).
     */
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier pivotSpeedSupplier,
            DoubleSupplier intakeRateSupplier, BooleanSupplier outtakeRateSupplier) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_pivotSpeedSupplier = pivotSpeedSupplier;
        this.m_intakeRateSupplier = intakeRateSupplier;
        this.m_outtakeRateSupplier = outtakeRateSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (Math.abs(m_intakeRateSupplier.getAsDouble()) > 0.05) {
            m_intakeSubsystem.intake(0.5);
        } else if (m_outtakeRateSupplier.getAsBoolean()) {
            m_intakeSubsystem.intake(-0.5);
        }
        m_intakeSubsystem.rotate(m_pivotSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake(0);
        m_intakeSubsystem.rotate(0);
    }
}