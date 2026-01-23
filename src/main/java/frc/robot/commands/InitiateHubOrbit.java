// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InitiateHubOrbit extends Command {

  private CommandSwerveDrivetrain dt;
  private SwerveRequest.RobotCentric drive;

  private PIDController rotPID = new PIDController(5, 0, 0);

  /** Readies chassis to orbit the hub. 
   * @param dt The drivetrain subsystem
   * @param radius The desired orbit radius
   * @param speed The desired orbit speed
  */
  public InitiateHubOrbit(CommandSwerveDrivetrain dt) {
    this.dt = dt;

    drive = new SwerveRequest.RobotCentric();

    rotPID.enableContinuousInput(-180, 180);
    rotPID.setTolerance(5);

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
    double angleToHub = Math.toDegrees(Math.atan2(hubCenter.getY() - robotPose.getY(), hubCenter.getX() - robotPose.getX()));
    double robotHeading = dt.getState().Pose.getRotation().getDegrees();
    
    
    dt.setControl(drive
    .withRotationalRate(DegreesPerSecond.of(rotPID.calculate(robotHeading, angleToHub))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotPID.atSetpoint();
  }
}
