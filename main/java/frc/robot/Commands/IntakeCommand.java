// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.BNASubystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  Intake sub;
  public IntakeCommand(Intake ezequiel) {
    this.sub = ezequiel;
    addRequirements(ezequiel);
  }

  @Override
  public void initialize() {
    sub.setSpeed(0.37);
  }


  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {
    sub.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}