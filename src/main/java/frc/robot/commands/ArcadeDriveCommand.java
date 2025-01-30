package frc.robot.commands;

import frc.robot.subsystems.DriveBaseSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class ArcadeDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // The drive base subsystem
  private final DriveBaseSubsystem driveBaseSubsystem;
  // The forward and backward speed
  private final Supplier<Double> forwardSpeedFunction, backwardSpeedFunction/*, rotationFunction */;
  // The X and Y values of a controler joystick
  private final Supplier<Double> xAxisFunction, yAxisFunction;

  // Creates a new ArcadeDriveCommand
  public ArcadeDriveCommand(DriveBaseSubsystem driveBaseSubsystem, Supplier<Double> forwardSpeedFunction, Supplier<Double> backwardSpeedFunction, Supplier<Double> xAxisFunction, Supplier<Double> yAxisFunction/*, Supplier<Double> rotationFunction */) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.forwardSpeedFunction = forwardSpeedFunction;
    this.backwardSpeedFunction = backwardSpeedFunction;
    this.xAxisFunction = xAxisFunction;
    this.yAxisFunction = yAxisFunction;
    //this.rotationFunction = rotationFunction
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Drives the robot using fieldcentric centric drive
  @Override
  public void execute() {
    // Gets the forward speed backwards speed from the suppliers
    double forwardSpeed = forwardSpeedFunction.get();
    double backwardSpeed = backwardSpeedFunction.get();
    // double rotation = rotationFunction.get();
    
    // Converts the X and Y values of the controller to an angle
    double xAxis = xAxisFunction.get();
    double yAxis = yAxisFunction.get() * -1;
    double angle = Math.toDegrees(Math.atan2(xAxis, yAxis));

    // Gets the rotation speed based on the the joystick heading and whether or not the robotis driving backwards
    double rotation = 0;
    if(xAxis > 0 || xAxis < 0 || yAxis > 0 || yAxis < 0) {
      rotation = driveBaseSubsystem.angleToRotation(angle, backwardSpeed > 0);
    }

    // Drives the robot either forward or backwards back on the whether or not the left trigger is pressed
    if(backwardSpeed > 0) {
      driveBaseSubsystem.driveMotors(backwardSpeed, rotation);
    } else {
      driveBaseSubsystem.driveMotors(forwardSpeed * -1, rotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
