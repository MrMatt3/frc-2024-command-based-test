package frc.robot.commands;

import frc.robot.Constants.OutTakeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class EjectCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // The intake subsystem
  private final IntakeSubsystem intakeSubsystem;
  // The shooter subsystem
  private final ShooterSubsystem shooterSubsystem;

  // Creates a new EjectCommand
  public EjectCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Ejects the note out of the robot by driving all the motors backwards
  @Override
  public void execute() {
    shooterSubsystem.setShooter(OutTakeConstants.ejectSpeed);
    shooterSubsystem.setMidMotor(OutTakeConstants.ejectSpeed);
    intakeSubsystem.setMotor(IntakeConstants.ejectSpeed);
  }

  // Stops all the motors when the eject command ends
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooter(0);
    shooterSubsystem.setMidMotor(0);
    intakeSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
