package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;

public class DriveBaseSubsystem extends SubsystemBase {
    // Motor controllers for the drive base
    private final SparkMax flMotor = new SparkMax(DriveConstants.frontLeftMotorPort, MotorType.kBrushless);
    private final SparkMax frMotor = new SparkMax(DriveConstants.frontRightMotorPort, MotorType.kBrushless);
    private final SparkMax blMotor = new SparkMax(DriveConstants.backLeftMotorPort, MotorType.kBrushless);
    private final SparkMax brMotor = new SparkMax(DriveConstants.backRightMotorPort, MotorType.kBrushless);
    // Differential drive base object made of the front left and right motors
    private final DifferentialDrive d_drive = new DifferentialDrive(flMotor, frMotor);
    // Gyroscope from the ADIS16470 IMU
    private final ADIS16470_IMU gyroscope = new ADIS16470_IMU();
    // PID for fieldcentric driving
    private final PIDController PID = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

    // Creates a new ShooterSubsystem.
    public DriveBaseSubsystem() {
        setMotorIdleModes();
        setCurrentLimits();
        setFollowers();
        configurePID();
    }

    // Makes the PID continuous at 0/360 and sets the tolerance to 2
    private void configurePID() {
        PID.enableContinuousInput(-180, 180);
        PID.setTolerance(PIDConstants.tolerance);
    }

    // Makes the back left and right motors followers of the front right and left motors
    private void setFollowers() {
        SparkMaxConfig leftFollow = new SparkMaxConfig();
        SparkMaxConfig rightFollow = new SparkMaxConfig();

        leftFollow.follow(flMotor);
        rightFollow.follow(frMotor);

        blMotor.configure(leftFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(rightFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets the motors to brake mode
    private void setMotorIdleModes() {
        SparkMaxConfig idleMode = new SparkMaxConfig();
        idleMode.idleMode(IdleMode.kBrake);

        frMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        blMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets the current limit of the motors to 40 amps
    private void setCurrentLimits() {
        SparkMaxConfig currentLimit = new SparkMaxConfig();
        currentLimit.smartCurrentLimit(DriveConstants.currentLimit);

        frMotor.configure(currentLimit, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flMotor.configure(currentLimit, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(currentLimit, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        blMotor.configure(currentLimit, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Drives the robot forward/backwards at a specified speed and turns at a specified turn speed
    public void driveMotors(double speed, double rotation) {
        d_drive.arcadeDrive(rotation, speed);
    }

    // Returns the robot heading (0 - 180/-180) from the gyroscope and gets the mirror value if the robot is driving backwards
    public double getAngle(boolean backwards) {
        double gyroscopeAngle = gyroscope.getAngle() * -1;
        if(backwards) {
            gyroscopeAngle = (gyroscopeAngle + 180) % 360;
        }
        if(gyroscopeAngle % 360  > 180) {
            return (gyroscopeAngle % 360) - 360;
        } else if (gyroscopeAngle % 360 < -180) {
            return (gyroscopeAngle % 360) + 360;
        } else {
            return gyroscopeAngle % 360;
        }
    }

    public double getRawAngle() {
        return gyroscope.getAngle();
    }

    // Returns an amount of motor effort/speed to turn based on the distance between the robot heading and a target point (0-360°) using the PID
    public double angleToRotation(double target, boolean backwards) {
        return PID.calculate(getAngle(backwards), target);
    }
}
