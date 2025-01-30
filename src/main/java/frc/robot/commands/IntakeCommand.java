package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // The intake subsystem
  private final IntakeSubsystem intakeSubsystem;
  // The intake speed
  private final Supplier<Double> speedFunction;

  // Creates a new IntakeCommand
  public IntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Double> speedFunction) {
    this.intakeSubsystem = intakeSubsystem;
    this.speedFunction = speedFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Intakes the note by driving the intake motors and motor in the middle of the launcher
  @Override
  public void execute() {
    intakeSubsystem.setMotor(speedFunction.get());
  }

  // Stops all the motors when the intake command ends
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
