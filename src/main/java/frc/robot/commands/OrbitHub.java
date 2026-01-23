// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OrbitHub extends Command {
  private Distance radius;
  private LinearVelocity speed;

  private CommandSwerveDrivetrain dt;
  private SwerveRequest.RobotCentric drive;

  private PIDController rotPID = new PIDController(5, 0, 0);
  private PIDController xPID = new PIDController(2, 0, 0);

  /** Creates a new OrbitHub. */
  public OrbitHub(CommandSwerveDrivetrain dt, Distance radius, LinearVelocity speed) {
    this.dt = dt;
    this.radius = radius;
    this.speed = speed;

    drive = new SwerveRequest.RobotCentric();

    rotPID.enableContinuousInput(-180, 180);

    addRequirements(this.dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotPose = dt.getState().Pose.getTranslation();
    Translation2d hubCenter = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    double robotToHubDist = robotPose.getDistance(hubCenter);
    double angleToHub = Math.toDegrees(Math.atan2(hubCenter.getY() - robotPose.getY(), hubCenter.getX() - robotPose.getX()));
    double robotHeading = dt.getState().Pose.getRotation().getDegrees();
    
    
    dt.setControl(drive
    .withVelocityX(-xPID.calculate(robotToHubDist, radius.in(Meters)))
    .withVelocityY(speed)
    .withRotationalRate(DegreesPerSecond.of(rotPID.calculate(robotHeading, angleToHub))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
