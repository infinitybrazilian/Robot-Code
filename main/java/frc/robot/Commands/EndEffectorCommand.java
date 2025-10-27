// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.BNASubystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  EndEffector subs;
  public EndEffectorCommand(EndEffector pistola) {
    this.subs = pistola;
    addRequirements(pistola);

  }

  @Override
  public void initialize() {
    subs.setSpeed(1.0);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    subs.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
