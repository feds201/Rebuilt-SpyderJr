package frc.robot.subsystems.intake;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.utils.LimelightHelpers;

public class DriveToBall extends Command {
  private CommandSwerveDrivetrain dt;
  private final PIDController hubRotPID = new PIDController(1, 0, 0);
  private SwerveRequest.RobotCentric driveNormal;

   public enum DriveToBallState {
    APPROACHING, 
    ALIGNING
  }


 DriveToBallState activeState = DriveToBallState.APPROACHING;

  public DriveToBall(CommandSwerveDrivetrain dt) {
    this.dt = dt;
    addRequirements(this.dt);
    driveNormal = new SwerveRequest.RobotCentric();
  }

 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hubRotPID.setSetpoint(0.0);
  }

  @Override
  public void execute() {
    double xError = LimelightHelpers.getTX("limelight-one");
    double Tx = LimelightHelpers.getTX("limelight-one");
    double Ty = LimelightHelpers.getTY("limelight-one");
    double Ta = LimelightHelpers.getTA(("limelight-one"));
    boolean Tv = LimelightHelpers.getTV("limelight-one");

    // when Tx is left tx is positive, so it would be ); (-Tx/30)
    
     if (Tv == true) {
       activeState = DriveToBallState.ALIGNING; 
     
       if (xError <= 30 && xError >= -30) {
         dt.setControl(driveNormal
         .withVelocityX(1.5)); 
       }

       else {
        dt.setControl(driveNormal
        .withVelocityY(Tx/30));

       }
     }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


//.withRotationalRate(-hubRotPID.calculate(xError)));

