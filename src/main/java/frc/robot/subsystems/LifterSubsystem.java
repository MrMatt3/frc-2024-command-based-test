package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.LifterConstants;

public class LifterSubsystem extends SubsystemBase {
    // Motor controllers for the lifters
    private final TalonSRX leftLifterMotor = new TalonSRX(LifterConstants.leftMotorPort);
    private final TalonSRX rightLifterMotor = new TalonSRX(LifterConstants.rightMotorPort);

    // Creates a new LifterSubsystem, sets the motors back to their default settings, and sets the motors to brake mode
    public LifterSubsystem() {
        restoreMotorDefaults();
        setMotorIdleModes();
    }

    // Resets the motors back to their default settings
    private void restoreMotorDefaults() {
        leftLifterMotor.configFactoryDefault();
        rightLifterMotor.configFactoryDefault();
    }

    // Sets the motors to brake mode
    private void setMotorIdleModes() {
        leftLifterMotor.setNeutralMode(NeutralMode.Brake);
        rightLifterMotor.setNeutralMode(NeutralMode.Brake);
    }

    // Drives each motor at an individual specified speed
    public void setMotors(double leftSpeed, double rightSpeed) {
        leftLifterMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightLifterMotor.set(ControlMode.PercentOutput, rightSpeed);
    }
}
