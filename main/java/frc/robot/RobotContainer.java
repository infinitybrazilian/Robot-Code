// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.EndEffectorCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.BNASubystems.EndEffector;
import frc.robot.subsystems.drivetrain.BNASubystems.Intake;
import frc.robot.subsystems.drivetrain.BNASubystems.IntakeAngle;
import swervelib.SwerveInputStream;

public class RobotContainer {

    // ====== Subsystems ======
    public final Swerve swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final EndEffector pistola = new EndEffector();
    public final Intake ezequiel = new Intake();
    public final IntakeAngle m_IntakeAngle = new IntakeAngle();

    // ====== Driver Control ======
    private final CommandXboxController driverControl = new CommandXboxController(0);

    // ====== Auto Chooser ======
    private final SendableChooser<Command> autoChooser;

    // ====== Drive Streams ======
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
            swerve.getSwerveDrive(),
            () -> driverControl.getLeftY() * -1,
            () -> driverControl.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverControl.getRightX())
            .deadband(Drivetrain.deadband)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true)
            .scaleRotation(0.8);

    SwerveInputStream driveAngularVelocityInvert = SwerveInputStream.of(
            swerve.getSwerveDrive(),
            () -> driverControl.getLeftY() * -1,
            () -> driverControl.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverControl.getRightX())
            .deadband(Drivetrain.deadband)
            .scaleTranslation(1.0)
            .allianceRelativeControl(true)
            .scaleRotation(0.6);

    // ====== Constructor ======
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // SmartDashboard.putData("Pivot Subsystem", pivot); // <--- mostra no dashboard
        DriverStation.silenceJoystickConnectionWarning(true);
        m_IntakeAngle.setDefaultCommand(IntakePivotCommand(IntakeAngle, ezequiel, fall));

        configureBindings();
    }

    // ====== Bindings ======
    private void configureBindings() {
        // ====== Swerve base drive ======
        Command driveFieldOrientedVelocity = swerve.driveFieldOriented(driveAngularVelocity);
        Command driveInvert = swerve.driveFieldOriented(driveAngularVelocityInvert);

        swerve.setDefaultCommand(driveFieldOrientedVelocity);
        driverControl.x().toggleOnTrue(driveInvert);

        // ====== Reset gyro ======
        driverControl.a().onTrue(Commands.runOnce(() -> swerve.resetGyro()));

        // ====== Intake / EndEffector ======
        driverControl.x().whileTrue(new IntakeCommand(ezequiel));
        driverControl.y().whileTrue(new EndEffectorCommand(pistola));

        driverControl.leftBumper().onTrue(m_IntakeAngle.setSetpoint(Constants.ScorePose.INITAL));
        driverControl.rightBumper().onTrue(m_IntakeAngle.setSetpoint(Constants.ScorePose.FLOOR));
        driverControl.leftTrigger().onTrue(m_IntakeAngle.setSetpoint(Constants.ScorePose.FLOAT));
        
        driverControl.b().whileTrue(new IntakePivotCommand(m_IntakeAngle, ezequiel, arise));

        // ====== Pivot subqqsystem control ======
//         driverControl.leftTrigger().onTrue(pivot.setSetpoint(PivotConstants.ScorePose.FLOOR));   // desce
//         driverControl.rightTrigger().onTrue(pivot.setSetpoint(PivotConstants.ScorePose.MIDDLE));  // meio
//         driverControl.leftBumper().onTrue(pivot.setSetpoint(PivotConstants.ScorePose.INITAL));  // volta posição inicial
//     }
    }
    // ====== Autonomous ======
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // ====== Utility ======
    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }

    public void setHeadingCorrection(boolean heading) {
        swerve.getSwerveDrive().setHeadingCorrection(heading);
    }
}
