package frc.robot.Commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends Command{
    private final ClimberSubsystem m_climberSubsystem;

    private final DoubleSupplier m_climberRate;

    public DefaultClimberCommand(ClimberSubsystem climberSubsystem, DoubleSupplier climberRate)
    {
        this.m_climberSubsystem = climberSubsystem;
        this.m_climberRate = climberRate;

        addRequirements(climberSubsystem);
    }

    @Override 
    public void execute(){
        m_climberSubsystem.rotate(m_climberRate.getAsDouble());
    }
   
    @Override 
    public void end(boolean interrupted){
        m_climberSubsystem.rotate(0);
    }
}
