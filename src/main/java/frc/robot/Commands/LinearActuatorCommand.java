package frc.robot.Commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.LinearActuatorSubsystem;

public class LinearActuatorCommand extends Command{
    private final LinearActuatorSubsystem m_actuatorSubsystem;

    private final DoubleSupplier m_actuatorRate;

    public LinearActuatorCommand(LinearActuatorSubsystem actuatorSubsystem, DoubleSupplier rateSupplier)
    {
        this.m_actuatorSubsystem = actuatorSubsystem;
        this.m_actuatorRate = rateSupplier;

        addRequirements(actuatorSubsystem);
    }

    @Override 
    public void execute(){
        if(Math.abs(this.m_actuatorRate.getAsDouble()) > .05)
        {
            m_actuatorSubsystem.actuate(-this.m_actuatorRate.getAsDouble() * .5);
        }
        else
        {
            m_actuatorSubsystem.actuate(0);
        }

    }
   
    @Override 
    public void end(boolean interrupted){
        m_actuatorSubsystem.actuate(0);
    }
}
