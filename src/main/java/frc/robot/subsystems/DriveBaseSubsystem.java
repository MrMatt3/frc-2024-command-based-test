package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
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
    // Encoders of the front left and right motors
    private final RelativeEncoder lEncoder = flMotor.getEncoder();
    private final RelativeEncoder rEncoder = frMotor.getEncoder();
    // Differential drive base object made of the front left and right motors
    private final DifferentialDrive d_drive = new DifferentialDrive(flMotor, frMotor);
    // Gyroscope from the ADIS16470 IMU
    private final ADIS16470_IMU gyroscope = new ADIS16470_IMU();
    // PID for note alignment
    private final PIDController alignPID = new PIDController(PIDConstants.alignKP, PIDConstants.alignKI, PIDConstants.alignKD);
    // PID for fieldcentric driving
    private final PIDController drivePID = new PIDController(PIDConstants.driveKP, PIDConstants.driveKI, PIDConstants.driveKD);
    // Pose estimator
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
        DriveConstants.kinematics,
        Rotation2d.fromDegrees(getAngle(false)),
        lEncoder.getPosition() * DriveConstants.rotationToMeters,
        rEncoder.getPosition() * DriveConstants.rotationToMeters,
        new Pose2d()
    );

    // Creates a new ShooterSubsystem.
    public DriveBaseSubsystem() {
        setMotorIdleModes();
        setCurrentLimits();
        setFollowers();
        configurePID();
        configureEncoders();
    }

    // Sets the conversion factor and resets the positions of the encoders back to 0
    public void configureEncoders() {
        SparkMaxConfig conversionFactor = new SparkMaxConfig();
        conversionFactor.encoder.positionConversionFactor(DriveConstants.rotationToMeters);
        flMotor.configure(conversionFactor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frMotor.configure(conversionFactor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        lEncoder.setPosition(0);
        rEncoder.setPosition(0);
    }

    // Makes the PID continuous at -180/180 and sets the tolerance to 2
    private void configurePID() {
        drivePID.enableContinuousInput(-180, 180);
        drivePID.setTolerance(PIDConstants.tolerance);
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

    // Returns the raw value from the gyroscope
    public double getRawAngle() {
        return gyroscope.getAngle();
    }

    // Returns the drive PID
    public PIDController getDrivePID() {
        return drivePID;
    }

    // Returns the note alignment PID
    public PIDController getAlignPID() {
        return alignPID;
    } 

    // Returns an amount of motor effort/speed to turn based on the distance between the robot heading and a target point (0-360°) using the PID
    public double angleToRotation(double target, boolean backwards) {
        return drivePID.calculate(getAngle(backwards), target);
    }

    // Returns the position of the robot
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // Periodically updates the pose estimator with odometry
    @Override
    public void periodic() {
      poseEstimator.update(
        Rotation2d.fromDegrees(getAngle(false)),
        lEncoder.getPosition(),
        rEncoder.getPosition() * -1
      );

      SmartDashboard.putNumber("X", poseEstimator.getEstimatedPosition().getX());
      SmartDashboard.putNumber("Y", poseEstimator.getEstimatedPosition().getY());
      SmartDashboard.putNumber("Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

      SmartDashboard.putNumber("Left Encoder Rotations", lEncoder.getPosition());
      SmartDashboard.putNumber("Right Encoder Position", rEncoder.getPosition());
    }
}
