package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;


public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable limelight;


  public LimelightSubsystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {


    super.periodic();
  }

}
