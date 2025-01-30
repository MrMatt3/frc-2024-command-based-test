package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  // Motor controller for the intake
  private final TalonSRX intakeMotor = new TalonSRX(IntakeConstants.motorPort);

  // Creates a new IntakeSubsystem
  public IntakeSubsystem() {}

  // Drives a motor at a specified speed
  public void setMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
}