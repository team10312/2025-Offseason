package frc.robot.generated;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.LimelightHelpers;

public class Constants {

    //Algae Arm
    public static final int kAlgaeArmMotorID = 2;
    public static final int mArmGearRatio = 36/1;

     // PPHolonomicDriveController gains
     public static final double TRANS_KP = 1.6, TRANS_KI = 0.0, TRANS_KD = 0.06;
     public static final double ROT_KP   = 4.0, ROT_KI   = 0.0, ROT_KD   = 0.15;

    //Pathfinding
    public static String limelightName = "limelight-low";
    public static final PathConstraints constraints = new PathConstraints(1, 1.5,Units.degreesToRadians(540), Units.degreesToRadians(720));
    public static final double aprilTagTolerance = 0.7;
}