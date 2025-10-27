package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

    int[] validIDS;
    private final SwerveDrive swerveDrive;

    // ConfigAuto configAuto;
    RobotConfig config;
    private Field2d field = new Field2d();
    boolean doRejectUpdate = false;
    PIDController pidAngle = new PIDController(1.8, 0.0, 0.0);

    public Swerve(File directory) {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {

            swerveDrive = new SwerveParser(directory).createSwerveDrive(Drivetrain.maxSpeed,
                    new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
                            Rotation2d.fromDegrees(0)));

            config = RobotConfig.fromGUISettings();

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, false, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();

        setupPathPlanner();
    }

    public Swerve(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Drivetrain.maxSpeed,
                new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                        Rotation2d.fromDegrees(0)));
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        SmartDashboard.putData(field);
    }

    public double[] botPose(String entry) {

        return NetworkTableInstance.getDefault().getTable("").getEntry(entry).getDoubleArray(new double[6]);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolomonicPose) {
        swerveDrive.resetOdometry(initialHolomonicPose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public void resetGyro() {
        swerveDrive.zeroGyro();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public Rotation2d getYaw() {
        return swerveDrive.getOdometryHeading();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            resetGyro();
            // Set the pose to 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            resetGyro();
        }
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY,
            double maxSpeed) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                maxSpeed);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle, double maxSpeed) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                maxSpeed);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public SwerveDriveConfiguration getSwerveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(pathName);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(1.5, 0.0, 0.0),
                            new PIDConstants(1.5, 0.0, 0.0)),
                    config,
                    () -> {

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

        } catch (Exception e) {
            e.printStackTrace();
        }

        PathfindingCommand.warmupCommand().schedule();
    }

}
