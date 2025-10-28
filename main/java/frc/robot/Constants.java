// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

public final class Constants {
  public static class Drivetrain {
    public static final int pdhID = 50;

    public static final double maxSpeed = 3; // 4.72
    public static final double loopTime = 0.13;
    public static final double robotMass = 108.03;

    public static final double turnConstant = 6;

    public static final double wheelDiameterInMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 21.43;
    public static final int encoderResolution = 1;

    public static final Matter chassi = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), robotMass);

    public static final double driveMotorConversion = SwerveMath.calculateMetersPerRotation(wheelDiameterInMeters,
        driveGearRatio, encoderResolution);
    public static final double steeringMotorConversion = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio,
        encoderResolution);
    public static final double deadband = 0.1;

    public static final double camPitch = 60;

    public static final double multiplicadorRotacional = 0.8;
    public static final double multiplicadorTranslacionalY = 0.9;
    public static final double multiplicadorTranslacionalX = 0.9;
  }

  public static final class Setpoints {
    public static final double intakeAriseSet = 0.0;
    public static final double intakeFallSet = 0.0;
  }

  public static final class PIDConstants {
    public static final double ariseP = 0.1;
    public static final double ariseI = 0.0;
    public static final double ariseD = 0.0;

    public static final double fallP = 0.1;
    public static final double fallI = 0.0;
    public static final double fallD = 0.0;
  }

  public static final class Strings {
    public static final String ON = "ON";
    public static final String OFF = "OFF";
  }
  // quando passar para o pc de programação lembra de criar um arquivo só pra isso
  public enum ScorePose {
    INITAL(0),  // Posição da Inspeção
    FLOOR(0),   // Posição para coleta dos elementos
    FLOAT(0);   // Posição para lançar a boia

    public final double angle;

    ScorePose(double angle) {
        this.angle = angle;
    }

}
}