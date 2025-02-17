package frc.robot.commands;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.Limelight;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // The drivebase subsystem
  private final DriveBaseSubsystem driveBaseSubsystem;
  // The speed of the robot
  private final Supplier<Double> speedFunction;

  // Creates a new EjectCommand
  public AutoAlignCommand(DriveBaseSubsystem driveBaseSubsystem, Limelight limelight, Supplier<Double> speedFunction) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.speedFunction = speedFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Aligns to the drive base pid and values from the limelight
  @Override
  public void execute() {
    if(Limelight.getTv() == 1) {
        driveBaseSubsystem.driveMotors(speedFunction.get(), -1 * driveBaseSubsystem.getAlignPID().calculate(Limelight.getTx()));
    }
  }

  // Stops all the motors when the eject command ends
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
