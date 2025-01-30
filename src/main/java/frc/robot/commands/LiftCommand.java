// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LifterSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class LiftCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // The lifter subsystem
  private final LifterSubsystem lifterSubsystem;
  // The left and right lifting speeds
  private final Supplier<Double> leftSpeedFunction, rightSpeedFunction;

  // Creates a new LiftCommand
  public LiftCommand(LifterSubsystem lifterSubsystem, Supplier<Double> leftSpeedFunction, Supplier<Double> rightSpeedFunction) {
    this.lifterSubsystem = lifterSubsystem;
    this.leftSpeedFunction = leftSpeedFunction;
    this.rightSpeedFunction = rightSpeedFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lifterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Makes the robot climb by driving left and right lifter motors
  @Override
  public void execute() {
    lifterSubsystem.setMotors(leftSpeedFunction.get(), rightSpeedFunction.get());
  }

  // Stops all the motors when the lift command ends
  @Override
  public void end(boolean interrupted) {
    lifterSubsystem.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
