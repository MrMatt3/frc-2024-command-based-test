package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// import frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.EjectCommand;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // All of the robot subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final LifterSubsystem lifterSubsystem = new LifterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // The driver and manipulator controllers
  private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driveControllerPort);
  private final CommandXboxController manipulatorController = new CommandXboxController(ControllerConstants.manipulatorController);

  // Sets the default commands of the drive base and lifter subsystems to the driving and lifting commands and sets the controller bindings
  public RobotContainer() {
    driveBaseSubsystem.setDefaultCommand(new ArcadeDriveCommand(driveBaseSubsystem, () -> driverController.getRightTriggerAxis(), () -> driverController.getLeftTriggerAxis(), () -> driverController.getLeftX(), () -> driverController.getLeftY()));
    lifterSubsystem.setDefaultCommand(new LiftCommand(lifterSubsystem, () -> manipulatorController.getLeftY(), () -> manipulatorController.getRightY()));
    configureBindings();
  }

  // Sets the controller bindings
  private void configureBindings() {
    // R Trigger - Shooting
    manipulatorController.rightTrigger(0.1).whileTrue(new ShootCommand(shooterSubsystem, () -> manipulatorController.getRightTriggerAxis()));
    // L Trigger - Intaking
    manipulatorController.leftTrigger(0.1).whileTrue(new IntakeCommand(intakeSubsystem, () -> manipulatorController.getLeftTriggerAxis()));
    // L Bumber - Ejecting
    manipulatorController.leftBumper().whileTrue(new EjectCommand(intakeSubsystem, shooterSubsystem));
  }
  // Returns the command to run during autonomous
  public Command getAutonomousCommand() {
    // return new ParallelCommandGroup(
    //   new ArcadeDriveCommand(driveBaseSubsystem, () -> 0.3, () -> 0.0, () -> 0.0, () -> 0.0).withTimeout(3),
    //   new IntakeCommand(intakeSubsystem, () -> 1.0).withTimeout(3),
    //   new ShootCommand(shooterSubsystem, () -> 1.0).withTimeout(3)
    // );
    return new ArcadeDriveCommand(driveBaseSubsystem, () -> 0.3, () -> 0.0, () -> 0.0, () -> 0.0).withTimeout(4);
  }
}
