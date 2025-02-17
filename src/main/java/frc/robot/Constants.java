package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class DriveConstants {
    public static final int frontLeftMotorPort = 1;
    public static final int frontRightMotorPort = 2;
    public static final int backRightMotorPort = 3;
    public static final int backLeftMotorPort = 4;

    public static final double turnSpeed = 0.6;
    public static final double slowSpeed = 0.3;
    public static final double slowTurnSpeed = 0.5;

    public static final int currentLimit = 40;

    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.5588);
    public static final double rotationToMeters = 1/17.964;
  }

  public static class PIDConstants {
    public static final double driveKP = 0.008;
    public static final double driveKI = 0.001;
    public static final double driveKD = 0;

    public static final double alignKP = 0.0175;
    public static final double alignKI = 0.001;
    public static final double alignKD = 0;

    public static final double tolerance = 2;
  }

  public static class IntakeConstants {
    public static final int motorPort = 8;

    public static final double ejectSpeed = -0.4;
    public static final double intakeSpeed = 1.0;
  }

  public static class OutTakeConstants {
    public static final int midMotorPort = 5;
    public static final int bottomMotorPort = 6;
    public static final int topMotorPort = 7;

    public static final double midMotorSpeed = 0.2;
    public static final double ejectSpeed = -0.2;
  }

  public static class LifterConstants {
    public static final int rightMotorPort = 9;
    public static final int leftMotorPort = 10;
  }

  public static class ControllerConstants {
    public static final int manipulatorController = 0;
    public static final int driveControllerPort = 5;
  }

  public static class LimelightConstants {
    public static final double defaultTx = 0.0;
    public static final long defaultTv = 0;
    public static final double[] defaultRobotPose = new double[6];
  }
}
