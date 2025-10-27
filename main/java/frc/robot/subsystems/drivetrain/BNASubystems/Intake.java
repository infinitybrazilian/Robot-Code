// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.BNASubystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
TalonFX Kraken1;
  public Intake() {

    Kraken1 = new TalonFX(35);
  }

 public void setSpeed(double setSpeed){

Kraken1.set(setSpeed);

 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
