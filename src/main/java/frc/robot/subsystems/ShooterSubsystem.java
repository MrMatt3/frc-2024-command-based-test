package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.OutTakeConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Motor controllers for the shooter
    private final SparkMax topMotor = new SparkMax(OutTakeConstants.topMotorPort, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(OutTakeConstants.bottomMotorPort, MotorType.kBrushless);
    private final SparkMax midMotor = new SparkMax(OutTakeConstants.midMotorPort, MotorType.kBrushless);

    // Creates a new ShooterSubsystem
    public ShooterSubsystem() {}

    // Drives the motors at the end of the launcher at a specified speed
    public void setShooter(double speed) {
        bottomMotor.set(speed);
        topMotor.set(speed);
    }

    // Drives the motors in the middle of the launcher at a specified speed
    public void setMidMotor(double speed) {
        midMotor.set(speed);
    }
}
