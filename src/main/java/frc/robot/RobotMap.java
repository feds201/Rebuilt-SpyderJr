package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.reflect.Field;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.SwerveModuleStatusUtil;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring.
 */
public final class RobotMap {


    public static class VisionConstants{
        //Use only yaw from MT1
        public static final Matrix<N3, N1> MT1_STDDEV = VecBuilder.fill(1e6, 1e6, Math.PI/60);
        //Use only x/y from MT2
        public static final Matrix<N3, N1> MT2_STDDEV = VecBuilder.fill(0.5, 0.5, 1e6);

    }

     public static final class IntakeSubsystemConstants {
        public static final int kMotorID = 1;
        public static final int kLimit_switch_rID = 2;
        public static final int kLimit_switch_lID = 3; 

    }

    public static class ShooterConstants {
        public static final int ShooterRightTop = 53;
        public static final int ShooterRightBottom = 52;
        public static final int ShooterBottomLeft = 51;
        public static final int ShooterTopLeft = 50;
        public static final int ShooterHood = 54;
        public static final AngularVelocity velocityTolerance = RotationsPerSecond.of(3);
         public static final Angle postionTolerance = Rotations.of(.05);

         public static final Angle maxHoodAngle = Rotations.of(27); //tune
         public static final Angle minHoodAngle = Rotations.of(0); //tune

        //offset of the shooter from robot center
        public static final Translation2d robotShooterOffset = new Translation2d(.25, 0); //TODO: tune
        //rotation of the shooter relative to robot forward
        public static final Rotation2d robotToShooterRotation = Rotation2d.fromDegrees(0.0);
        public static final Translation2d hubCenter = FieldConstants.Hub.innerCenterPoint.toTranslation2d();   
        public static final Rectangle2d trench = new Rectangle2d(robotShooterOffset, hubCenter);
        public static final Translation2d passingRight = FieldConstants.Outpost.centerPoint.plus(new Translation2d(0, 2));
        public static final Translation2d passingLeft = new Translation2d(0, 7.44).minus(new Translation2d(0, 2));
        public static final Translation2d BlueLeftTopLeft = new Translation2d(4.0, 8.208);
        public static final Translation2d BlueLeftBottomRight = new Translation2d(5.17, 6.75);
        public static final Rectangle2d BlueLeftTrench = new Rectangle2d(BlueLeftTopLeft, BlueLeftBottomRight);

        public static final Translation2d RedLeftTopLeft = new Translation2d(11.375, 1.221);
        public static final Translation2d RedLeftBottomRight = new Translation2d(12.6, 0.082);
        public static final Rectangle2d RedLeftTrench = new Rectangle2d(RedLeftTopLeft, RedLeftBottomRight);

        public static final Translation2d BlueRightTopLeft = new Translation2d(5.2, 0.018);
        public static final Translation2d BlueRightBottomRight = new Translation2d(4, 1.26);
        public static final Rectangle2d BlueRightTrench = new Rectangle2d(BlueRightTopLeft, BlueRightBottomRight);

        public static final Translation2d RedRightTopRight = new Translation2d(12.56, 6.88);
        public static final Translation2d RedRightBottomRight = new Translation2d(11.181, 8.104);
        public static final Rectangle2d RedRightTrench = new Rectangle2d(RedRightTopRight, RedRightBottomRight);

        public static final Rectangle2d neutralZone = new Rectangle2d(FieldConstants.LeftTrench.openingTopLeft.toTranslation2d(), FieldConstants.RightTrench.oppOpeningTopRight.toTranslation2d());
    
        // This map is used to determine the velocity of the shooter based on the distance to the target. 
        //The key is the distance to the target in meters, and the value is the velocity of the shooter in rotations per second.`
        public static final InterpolatingDoubleTreeMap kShootingVelocityMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.44, 26.5),//done - changed by 0.5
            Map.entry(1.7, 26.0),//done - changed by 0.5
            Map.entry(2.01, 25.9),//done - changed by 0.10
            Map.entry(2.56, 28.7),//done - changed by 0.8
            Map.entry(2.89, 29.0),//done - changed by 0.5
             Map.entry(3.08, 31.0),//done --- AUTON SHOOTING POSITION
            Map.entry(3.37, 33.5),
            Map.entry(3.97,40.0), //done - increased by 0.5
            Map.entry(4.75, 38.0),//done - increase by 0.5
            Map.entry(100.0, 40.0)//far off top limit to prevent unwanted scaling past this distance 
        );

        public static final InterpolatingDoubleTreeMap kShootingPositionMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.44, 0.0),//done
            Map.entry(1.77, 0.0),//done
            Map.entry(2.01, 3.5),//done - changed by 0.3
            Map.entry(2.56, 8.5),//done - changed by 0.3
            Map.entry(2.89, 8.8),//done
             Map.entry(3.08, 7.3),// -- AUTON SHOOTING POSITION
            Map.entry(3.37, 7.4),
            Map.entry(3.97,7.3),//done
            Map.entry(4.75, 8.6), //done - increased by 0.3
            Map.entry(100.0, 9.8) //far off top limit to prevent unwanted scaling past this distance 
        );

        public static final InterpolatingDoubleTreeMap kPassingVelocityMap = InterpolatingDoubleTreeMap.ofEntries(
           Map.entry(5.07, 26.0),
           Map.entry(6.5, 30.0),
           Map.entry(8.53, 35.0),
           Map.entry(11.12, 44.0)
        );


         public static final InterpolatingDoubleTreeMap kPassingPositionMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(5.07, 29.0),
            Map.entry(6.5,29.0),
            Map.entry(8.53, 29.0),
            Map.entry(11.12, 29.0)
        );

         

        public static final InterpolatingDoubleTreeMap kFlightTimeMap =
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.5, .87),
            Map.entry(2.0, 0.93),
            Map.entry(2.5, 1.03),
            Map.entry(3.0, 1.09),
            Map.entry(3.5, 1.27),
            Map.entry(4.0, 1.8)
        );

        
    }

}
