// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.InitiateHubOrbit;
import frc.robot.commands.OrbitHub;
import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FuelSim;
import frc.robot.utils.LimelightWrapper;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final LimelightWrapper sampleLocalizationLimelight = new LimelightWrapper("limelight");

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

        
        configureBindings();
    }

      public void updateLocalization() {
    sampleLocalizationLimelight.getSettings()
        .withRobotOrientation(new Orientation3d(drivetrain.getRotation3d(),
            new AngularVelocity3d(DegreesPerSecond.of(drivetrain.getPigeon2().getAngularVelocityXWorld().getValueAsDouble()),
                DegreesPerSecond.of(drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble()),
                DegreesPerSecond.of(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()))))
        .save();

    // Get MegaTag2 pose
    Optional<limelight.networktables.PoseEstimate> visionEstimate = sampleLocalizationLimelight.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    // If the pose is present
    visionEstimate.ifPresent((limelight.networktables.PoseEstimate poseEstimate) -> {
      // Add it to the pose estimator.
      drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
    });

  }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, joystick));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

       //joystick.rightBumper().whileTrue(new InitiateHubOrbit(drivetrain).andThen(new OrbitHub(drivetrain, Meters.of(3), MetersPerSecond.of(1))));
        /* Manually start logging with left bumper before running any tests,
         * and stop logging with right bumper after we're done with ALL tests.
         * This isn't necessary but is convenient to reduce the size of the hoot file */
        // joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.povDown().onTrue(new InstantCommand(() -> FuelSim.getInstance().spawnFuel(new Translation3d(1, 1, 1), new Translation3d(0, 0, 0))));

        // Reset the field-centric heading on left bumper press.
        //joystick.leftBumper().whileTrue(new InitiateHubOrbit(drivetrain).andThen(new OrbitHub(drivetrain, Meters.of(3), MetersPerSecond.of(-1))));
        
        joystick.start().onTrue(new InstantCommand(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setUpFuelSim()
  {
 Shuffleboard.getTab("FuelSim").add(" pose", drivetrain.getState().Pose.getX());
    double width=30/36;//inches
    double length=30/36;//inches
    double bumperHeight=7/36;//inches
FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

// Register a robot for collision with fuel
FuelSim.getInstance().registerRobot(
        width, // from left to right
        length, // from front to back
        bumperHeight,
        drivetrain::getRobotPose, // Supplier<Pose2d> of robot pose,
        drivetrain::getChassisSpeeds
        );// from floor to top of bumpers
 // Supplier<ChassisSpeeds> of field-centric chassis speeds

FuelSim.getInstance().setSubticks(5);
FuelSim.getInstance().start(); // enables the simulation to run (updateSim must still be called periodically)
  }

}

