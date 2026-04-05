package frc.robot.subsystems.intake;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class DriveToBall extends Command {
  private CommandSwerveDrivetrain dt;
    private final PIDController hubRotPID = new PIDController(1, 0, 0);
     private SwerveRequest.RobotCentric driveNormal;
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
    boolean Tv = LimelightHelpers.getTV("limelight-one");

    if (Tv == false) {
        dt.setControl(driveNormal
          .withVelocityX(1) //arbitrary value, will need to be tunedd
          .withVelocityY(1)
          .withRotationalRate(0));
      }

     dt.setControl(driveNormal
          .withVelocityX(Tx/10)
          .withVelocityY(Ty/5)
          .withRotationalRate(-hubRotPID.calculate(xError)));

  }

 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double xError = LimelightHelpers.getTX("limelight-one");
    dt.setControl(driveNormal
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(-hubRotPID.calculate(xError)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
