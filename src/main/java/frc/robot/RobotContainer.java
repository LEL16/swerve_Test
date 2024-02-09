package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutonIntakeCommand;
import frc.robot.Commands.AutonOuttakeCommand;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.IdleDriveCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private double m_powerLimit = 1.0;

  private static final double kIntakeGearRatio = 1;
  public static final double kIntakeAutonRate = 5676.0 * kIntakeGearRatio * 0.5; // rpm

  private static final double kRotateGearRatio = (1.0 / 20.0);
  public static final double kRotateAutonAngularSpeed = 5676.0 * 0.5 * kRotateGearRatio * 2.0 * Math.PI / 60; // rad/s

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Field2d field;


  /**
   * This class stores all robot related subsystems, commands, and methods that
   * the {@link Robot} class can utilize during different OpModes.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> (-MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) / 2.0) * m_powerLimit
            * DrivetrainSubsystem.kMaxAngularSpeed
    ));

    /*
    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
        m_intakeSubsystem, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05) * IntakeSubsystem.kIntakeMaxRate, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.05) * IntakeSubsystem.kRotateMaxAngularSpeed
    ));
    */



    field = new Field2d();
    SmartDashboard.putData("Field", field);
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Add what we want to do with poses
      field.setRobotPose(pose);
    });
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Add what we want to do with poses
      field.getObject("target pose").setPose(pose);
    });
    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Add what we want to do with poses
      field.getObject("path").setPoses(poses);
    });

    autoChooser = AutoBuilder.buildAutoChooser("test"); // Default path
    SmartDashboard.putData("Auto Chooser", autoChooser);


    NamedCommands.registerCommand("defaultIntakeCommand", new AutonIntakeCommand(m_intakeSubsystem, kIntakeAutonRate, 2));
    NamedCommands.registerCommand("defaultOuttakeCommand", new AutonOuttakeCommand(m_outtakeSubsystem, kIntakeAutonRate, 2));

    configureButtons();





  }

  public Command getAutonomousCommand() {
    autoChooser = AutoBuilder.buildAutoChooser("test123"); // Default path
    SmartDashboard.putData("Auto Chooser", autoChooser); // Elastic path chooser
    return autoChooser.getSelected();
  }

  // Currently used for testing kinematics
  // public Command autonomousCommands() {
  //   m_powerLimit = 1.0;
  //   // m_intakeSubsystem.reset();
  //   return new SequentialCommandGroup(
  //     new PositionDriveCommand(m_drivetrainSubsystem, 1.0, 0.5, Math.PI / 2, 2.5, Math.PI, 1500),
  //     new PositionDriveCommand(m_drivetrainSubsystem, 2.0, 0, 0, 2.5, Math.PI, 1500),
  //     new PositionDriveCommand(m_drivetrainSubsystem, 1.0, -0.5, -Math.PI / 2, 2.5, Math.PI, 1500),
  //     new PositionDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 2.5, Math.PI, 1500)
  //   );
  // }

  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.onFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> (m_driveController.getPOV() >= 315
        || (m_driveController.getPOV() <= 45 && m_driveController.getPOV() >= 0)));
    m_incrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(0.2)));

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(
        () -> (m_driveController.getPOV() >= 135 && m_driveController.getPOV() <= 225));
    m_decrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(-0.2)));
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    m_drivetrainSubsystem.alignTurningEncoders();
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }
}
