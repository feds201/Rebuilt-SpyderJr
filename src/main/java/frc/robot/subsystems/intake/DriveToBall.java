package frc.robot.subsystems.intake;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class DriveToBall extends Command {
  private CommandSwerveDrivetrain dt;
  private final PIDController hubRotPID = new PIDController(0.1, 0, 0);
  private SwerveRequest.RobotCentric driveNormal;



  public DriveToBall(CommandSwerveDrivetrain dt) {
    this.dt = dt;
    addRequirements(this.dt);
    driveNormal = new SwerveRequest.RobotCentric();

    hubRotPID.setTolerance(1.0);
  }

 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hubRotPID.setSetpoint(0.0);
  }


   @Override
public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-one");

    if (hasTarget) {
        double tx = LimelightHelpers.getTX("limelight-one");
        double ty = LimelightHelpers.getTY("limelight-one");

        if (Math.abs(tx) < 1.5) tx = 0.0;

        double rotationOutput = -hubRotPID.calculate(tx);
        rotationOutput = Math.max(-2.0, Math.min(2.0, rotationOutput));

        
        double forwardVelocity = Math.abs((ty - 20) * 0.10);

        if (ty > -5) {
           dt.setControl(driveNormal
            .withVelocityX(forwardVelocity)
            .withVelocityY(0)
            .withRotationalRate(rotationOutput)
        );

        }

       
    } else {
        dt.setControl(driveNormal
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(3.2) //in radians per second
        );
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

