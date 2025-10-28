// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.BNASubystems.Intake;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePivotCommand extends Command {
  IntakeAngle intakeAngle;
  Intake intake;
  PIDController arisePID;
  PIDController fallPID;
  String state;

  public IntakeCommand(IntakeAngle intakeAngle, Intake intake, String state) {
    this.intakeAngle = intakeAngle;
    this.intake = intake;
    thsi.state = state;

    arisePID = new PIDController(PIDConstants.ariseP, PIDConstants.ariseI, PIDConstants.ariseD);
    fallPID = new PIDController(PIDConstants.fallP, PIDConstants.fallI, PIDConstants.fallD);
    addRequirements(intakeAngle, intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (state){
      case arise:
        ariseOutput = MathUtil.clamp(arisePID.calculate(IntakeAngle.getAngle, Setpoints.intakeAriseSet), -0.5, 0.5);
        IntakeAngle.setSpeed(ariseOutput);
        Intake.setSpeed(0.5);

      case fall:
        fallOutput = MathUtil.clamp(fallPID.calculate(IntakeAngle.getAngle, Setpoints.intakeFallSet), -0.5, 0.5);
        IntakeAngle.setSpeed(fallOutput);
        Intake.setSpeed(0);

      case default:
        fallOutput = MathUtil.clamp(fallPID.calculate(IntakeAngle.getAngle, Setpoints.intakeFallSet), -0.5, 0.5);
        IntakeAngle.setSpeed(fallOutput);
        Intake.setSpeed(0);
    }
  }

  
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}