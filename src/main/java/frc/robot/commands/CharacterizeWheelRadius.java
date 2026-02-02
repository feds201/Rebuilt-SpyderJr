// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CharacterizeWheelRadius extends Command {
  /** Creates a new CharacterizeWheelRadius. */
  private final CommandSwerveDrivetrain dt;
  //meters
  private final double chassisRadius = Math.sqrt(TunerConstants.FrontLeft.LocationX * TunerConstants.FrontLeft.LocationX + TunerConstants.FrontLeft.LocationY * TunerConstants.FrontLeft.LocationY);
  private final double chassisRotationCircumference = 2 * Math.PI * chassisRadius;
  //degrees
  private double initialChassisPos;
  //rotations
  private double initialDriveMotorPos;

  public CharacterizeWheelRadius(CommandSwerveDrivetrain drivetrain) {
    dt = drivetrain;
    
    
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialChassisPos = dt.getState().Pose.getRotation().getDegrees() + 180;
    initialDriveMotorPos = dt.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
    dt.setControl(new SwerveRequest.FieldCentric().withRotationalRate(DegreesPerSecond.of(90)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setControl(new SwerveRequest.Idle());
    double finalDriveMotorPos = dt.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
    double wheelRotations = (finalDriveMotorPos - initialDriveMotorPos) / TunerConstants.FrontLeft.DriveMotorGearRatio;
    double wheelCircumference = chassisRotationCircumference / wheelRotations;
    double wheelRadius = wheelCircumference / (2 * Math.PI);
    SmartDashboard.putNumber("Characterized Wheel Radius (m)", wheelRadius);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dt.getState().Pose.getRotation().getDegrees() == initialChassisPos;
  }
}
