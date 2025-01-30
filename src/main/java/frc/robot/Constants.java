// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
  }

  public static class PIDConstants {
    public static final double kP = 0.008;
    public static final double kI = 0.001;
    public static final double kD = 0;

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
}
