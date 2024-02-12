package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Drive.BrakeCommand;
import frc.robot.Commands.Drive.DefaultDriveCommand;
import frc.robot.Commands.Intake.DefaultIntakeCommand;
import frc.robot.Commands.Limelight.LimelightAlignmentCommand;
import frc.robot.Commands.Limelight.LimelightPathfindingCommand;
import frc.robot.Commands.Outtake.DefaultOuttakeCommand;
import frc.robot.Commands.Pivot.DefaultPivotCommand;
import frc.robot.Commands.Pivot.PositionPivotCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.PivotSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private final GenericHID m_operatorButtonPad = new GenericHID(2);

  private double m_powerLimit = 1.0;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Field2d m_field;

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
            * DrivetrainSubsystem.kMaxAngularSpeed));

    /* Controller implementation
    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
        m_intakeSubsystem,
        () -> m_operatorController.getRawButton(5),
        () -> m_operatorController.getRawButton(6)));

        */
    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
            m_intakeSubsystem,
            () -> m_operatorButtonPad.getRawButton(6),
            () -> m_operatorButtonPad.getRawButton(7)));

    /* Controller implementation
    m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
        m_outtakeSubsystem,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.01) * m_powerLimit));
    */

    m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
    m_outtakeSubsystem,
    () -> {
        boolean buttonPressed = m_operatorButtonPad.getRawButton(3);
        return buttonPressed ? 0.8 : 0.0; 
    }));

    m_pivotSubsystem.setDefaultCommand(new DefaultPivotCommand(
    m_pivotSubsystem,
    () -> {
        boolean upButtonPressed = m_operatorButtonPad.getRawButton(7);
        boolean downButtonPressed = m_operatorButtonPad.getRawButton(8);
        if(upButtonPressed) return 0.7;
        else if(downButtonPressed) return 0.7;
        return 0;
    }, false));

        
    m_pivotSubsystem.setDefaultCommand(new DefaultPivotCommand(
        m_pivotSubsystem,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05) * m_powerLimit,
        () -> m_operatorController.getRawButton(1)));

    m_field = new Field2d();

    ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

    autonomousTab.add("Field", m_field);
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_field.setRobotPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_field.getObject("path").setPoses(poses);
    });

    NamedCommands.registerCommand("Shoot Note", new InstantCommand(() -> m_outtakeSubsystem.outtakeRotate(0.5)));
    NamedCommands.registerCommand("Stop Shoot Note", new InstantCommand(() -> m_outtakeSubsystem.outtakeRotate(0.0)));
    NamedCommands.registerCommand("Intake Note", new InstantCommand(() -> m_intakeSubsystem.intakeRotate(0.5)));
    NamedCommands.registerCommand("Stop Intake Note", new InstantCommand(() -> m_intakeSubsystem.intakeRotate(0.0)));

    // NamedCommands.registerCommand("resetPos", new InstantCommand(() -> setPose(0,
    // 0, 0))); // Example registered command
    autoChooser = AutoBuilder.buildAutoChooser("DefaultAuton"); // Default path
    autonomousTab.add("Auto Chooser", autoChooser);

    configureButtons();
  }

  // Currently used for testing kinematics
  public Command autonomousCommands() {
    m_powerLimit = 1.0;
    // return new PathPlannerAuto("DefaultAuton"); // Debugging return statement

    // return new PositionDriveCommand(m_drivetrainSubsystem, 2.0, 0,
    // Math.toRadians(45), 3.66, 10.35);
    return autoChooser.getSelected();
  }

  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.whileFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver button B
    Trigger m_pathfinding = new Trigger(() -> m_driveController.getRawButton(2));
    m_pathfinding.onTrue(AutoBuilder.pathfindToPose(new Pose2d(8.30, 4.10, Rotation2d.fromDegrees(90)),
        new PathConstraints(2.0, 2.0, Units.degreesToRadians(180), Units.degreesToRadians(180)), 0.0, 0.0));

    // Driver button LB
    Trigger m_limelightPathFinding = new Trigger(() -> m_driveController.getRawButton(5));
    m_limelightPathFinding.onTrue(new LimelightPathfindingCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1));
    m_limelightPathFinding.whileFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver button RB
    Trigger m_limelightAlignment = new Trigger(() -> m_driveController.getRawButton(6));
    m_limelightAlignment.onTrue(new LimelightAlignmentCommand(m_drivetrainSubsystem, m_limelightSubsystem));
    m_limelightAlignment.whileFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> (m_driveController.getPOV() >= 315
        || (m_driveController.getPOV() <= 45 && m_driveController.getPOV() >= 0)));
    m_incrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(0.2)));

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(
        () -> (m_driveController.getPOV() >= 135 && m_driveController.getPOV() <= 225));
    m_decrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(-0.2)));

    // Operator button A
    Trigger m_pivotLowPosition = new Trigger(
        () -> m_operatorController.getRawButton(1));
    m_pivotLowPosition.whileTrue(new PositionPivotCommand(m_pivotSubsystem, "low"));
    m_pivotLowPosition.whileFalse(new InstantCommand(() -> m_pivotSubsystem.getCurrentCommand().cancel()));

    // Operator button B
    Trigger m_pivotMidPosition = new Trigger(
        () -> m_operatorController.getRawButton(2));
    m_pivotMidPosition.whileTrue(new PositionPivotCommand(m_pivotSubsystem, "mid"));
    m_pivotMidPosition.whileFalse(new InstantCommand(() -> m_pivotSubsystem.getCurrentCommand().cancel()));

    // Operator button X
    Trigger m_pivotHighPosition = new Trigger(
        () -> m_operatorController.getRawButton(4));
    m_pivotHighPosition.whileTrue(new PositionPivotCommand(m_pivotSubsystem, "high"));
    m_pivotHighPosition.whileFalse(new InstantCommand(() -> m_pivotSubsystem.getCurrentCommand().cancel()));


    //Buttons are NOT correctly assigned currently

    // Operator "low" position with button-pad button
    Trigger m_pivotLowPositionButtonPad = new Trigger(
        () -> m_operatorButtonPad.getRawButton(1));
    m_pivotLowPositionButtonPad.whileTrue(new PositionPivotCommand(m_pivotSubsystem, "low"));
    m_pivotLowPositionButtonPad.whileFalse(new InstantCommand(() -> m_pivotSubsystem.getCurrentCommand().cancel()));

    // Operator "mid" position with button-pad button
    Trigger m_pivotMidPositionButtonPad = new Trigger(
        () -> m_operatorButtonPad.getRawButton(2));
    m_pivotMidPositionButtonPad.whileTrue(new PositionPivotCommand(m_pivotSubsystem, "mid"));
    m_pivotMidPositionButtonPad.whileFalse(new InstantCommand(() -> m_pivotSubsystem.getCurrentCommand().cancel()));

    // Operator "high" position with button-pad button
    Trigger m_pivotHighPositionButtonPad = new Trigger(
        () -> m_operatorButtonPad.getRawButton(3));
    m_pivotHighPositionButtonPad.whileTrue(new PositionPivotCommand(m_pivotSubsystem, "high"));
    m_pivotHighPositionButtonPad.whileFalse(new InstantCommand(() -> m_pivotSubsystem.getCurrentCommand().cancel()));
   
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
  }

  public void setIdleMode(String idleMode) {
    m_drivetrainSubsystem.setIdleMode(idleMode);
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }
}