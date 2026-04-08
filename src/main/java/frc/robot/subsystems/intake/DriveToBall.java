package frc.robot.subsystems.intake;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class DriveToBall extends Command {
  private CommandSwerveDrivetrain dt;
  private final PIDController hubRotPID = new PIDController(0.1, 0, 0);
  private SwerveRequest.RobotCentric driveNormal;
  private double maxVelocity;
  private ArrayList<Double> tyHistory = new ArrayList<Double>(List.of(0.0, 0.0, 0.0, 0.0, 0.0));
  private double averageTy; 
  private boolean hasTarget;



  public DriveToBall(CommandSwerveDrivetrain dt) {
    this.dt = dt;
    addRequirements(this.dt);
    driveNormal = new SwerveRequest.RobotCentric();
    hubRotPID.setTolerance(1.0);
    maxVelocity = 2; 
  }

 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hubRotPID.setSetpoint(0.0);
  }


   @Override
public void execute() {
      hasTarget = LimelightHelpers.getTV("limelight-one");

    if (hasTarget) {
        double tx = LimelightHelpers.getTX("limelight-one");
        double ty = LimelightHelpers.getTY("limelight-one");

        if (Math.abs(tx) < 1.5) tx = 0.0;

        double rotationOutput = -hubRotPID.calculate(tx);
        rotationOutput = Math.max(-2.0, Math.min(2.0, rotationOutput));

        
        double forwardVelocity = Math.abs((ty - 23) * 0.1);

        if (forwardVelocity >= maxVelocity){
          forwardVelocity = averageTy; 
        }

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

public void AverageTy() {
if (hasTarget) {
   if (tyHistory.size() > 5) {
         tyHistory.remove(0);
    }

    if (tyHistory.size() == 5) {

      for (int i = 0; i < (tyHistory.size()); i++) {
        averageTy += tyHistory.get(i);
      }
       averageTy /= tyHistory.size();
        
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

