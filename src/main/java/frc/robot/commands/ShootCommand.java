package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OutTakeConstants;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // The shooter subsystem
  private final ShooterSubsystem shooterSubsystem;
  // The shooting speed
  private final Supplier<Double> speedFunction;

  // Creates a new ShootCommand
  public ShootCommand(ShooterSubsystem shooterSubsystem, Supplier<Double> speedFunction) {
    this.shooterSubsystem = shooterSubsystem;
    this.speedFunction = speedFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Shoots out the note by driving the motor in the middle of the shooter and the motors at the end of the shooter
  @Override
  public void execute() {
    shooterSubsystem.setMidMotor(OutTakeConstants.midMotorSpeed);
    shooterSubsystem.setShooter(speedFunction.get());
  }

  // Stops all the motors when the shoot command ends
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setMidMotor(0);
    shooterSubsystem.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
