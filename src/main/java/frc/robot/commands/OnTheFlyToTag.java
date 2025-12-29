package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

public class OnTheFlyToTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private static final PathConstraints kConstraints = new PathConstraints(
      2.0,                  // max velocity (m/s)
      3.0,                  // max accel (m/s^2)
      Math.toRadians(360),  // max angular velocity (rad/s)
      Math.toRadians(540)   // max angular accel (rad/s^2)
  );

  private static final double kStandoffDistance = 0.5; // meters

  // SIM: fixed AprilTag pose in field (You can change this to test different angles)
  private static final Pose2d kSimTagFieldPose =
      new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(60));

  private boolean inSimulation = false;
  private Pose2d targetPose = null;
  private Rotation2d targetRotation = null;
  private PathPlannerPath generatedPath = null;
  private List<PathPoint> allPathPoints = null;
  private Command followCmd = null;
  private boolean hadValidPath = false;
  private boolean turnedGreen = false;

  private final SwerveRequest.FieldCentric stopRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public OnTheFlyToTag(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    new AnimateLed(AnimationType.Rainbow).schedule();
    drivetrain.setDriveToTagRunning(true);
    inSimulation = Utils.isSimulation();

    SmartDashboard.putNumberArray("OnTheFly/PathPoints", new double[0]);
    SmartDashboard.putNumberArray("OnTheFly/TargetPose", new double[0]);

    generatedPath = null;
    allPathPoints = null;
    targetPose = null;
    targetRotation = null;
    followCmd = null;
    hadValidPath = false;
    turnedGreen = false;

    Pose2d startPose = drivetrain.getState().Pose;
    Pose2d rawTagFieldPose;

    // -------------------------------------------------------------------------
    // STEP 1: ACQUIRE RAW DATA (The only part that differs)
    // -------------------------------------------------------------------------
    if (inSimulation) {
      // SIM: Use Constant
      rawTagFieldPose = kSimTagFieldPose;
    } else {
      // REAL: Use Limelight
      if (!LimelightHelpers.getTV(Constants.limelightName)) {
        System.out.println("OnTheFly: No Tag Visible");
        return;
      }
      
      Pose2d tagRelative = drivetrain.getAprilTagPose();

      // COORDINATE FIX: Convert Limelight (Y-Right) to WPILib (Y-Left)
      // We invert Y and Invert Rotation to match the coordinate systems.
      Translation2d correctedTrans = new Translation2d(tagRelative.getX(), -tagRelative.getY());
      Rotation2d correctedRot = tagRelative.getRotation().unaryMinus();

      Transform2d robotToTag = new Transform2d(correctedTrans, correctedRot);
      rawTagFieldPose = startPose.transformBy(robotToTag);
    }

    // -------------------------------------------------------------------------
    // STEP 2: NORMALIZE ORIENTATION (Exact same logic for Sim and Real)
    // -------------------------------------------------------------------------
    // This ensures that "tagFieldPose" always represents a tag facing OUT towards the robot.
    // This fixes the issue where Sim works but Real Life spins because of 0 vs 180 confusion.
    
    Translation2d tagToRobot = startPose.getTranslation().minus(rawTagFieldPose.getTranslation());
    
    // Create a vector representing the direction the tag CLAIMS to be facing
    Translation2d tagFacingVec = new Translation2d(1.0, rawTagFieldPose.getRotation());

    // Dot Product: If (Tag->Robot) and (TagFacing) point in same direction, Dot > 0.
    double dotProduct = (tagToRobot.getX() * tagFacingVec.getX()) + 
                        (tagToRobot.getY() * tagFacingVec.getY());

    Pose2d finalTagPose = rawTagFieldPose;

    // If Dot Product is negative, the tag thinks it's facing AWAY from the robot (into the wall).
    // We flip it 180 degrees to fix it.
    if (dotProduct < 0) {
        finalTagPose = new Pose2d(
            rawTagFieldPose.getTranslation(),
            rawTagFieldPose.getRotation().plus(Rotation2d.k180deg)
        );
    }

    // -------------------------------------------------------------------------
    // STEP 3: CALCULATE TARGET (Exact same logic for Sim and Real)
    // -------------------------------------------------------------------------
    // We use finalTagPose.getRotation() to determine the approach angle.
    // This allows the robot to approach at an angle (like 60deg) if the tag is angled.
    
    Translation2d approachOffset = new Translation2d(kStandoffDistance, 0).rotateBy(finalTagPose.getRotation());
    Translation2d targetTrans = finalTagPose.getTranslation().plus(approachOffset);
    
    // Robot must look OPPOSITE to the tag's facing direction
    targetRotation = finalTagPose.getRotation().plus(Rotation2d.k180deg);
    
    targetPose = new Pose2d(targetTrans, targetRotation);

    // Logging
    SmartDashboard.putNumberArray("OnTheFly/TargetPose", new double[] {
        targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians()
    });

    // -------------------------------------------------------------------------
    // STEP 4: GENERATE PATH
    // -------------------------------------------------------------------------
    double dist = startPose.getTranslation().getDistance(targetPose.getTranslation());
    
    // Safety: If already there, stop.
    if (dist < 0.1) {
        turnedGreen = true;
        new SetLedColor(0, 255, 0).schedule();
        return; 
    }

    Rotation2d travelDir = targetPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    Translation2d midpoint = startPose.getTranslation().plus(targetPose.getTranslation()).div(2.0);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(startPose.getTranslation(), travelDir),
        new Pose2d(midpoint, travelDir),
        new Pose2d(targetPose.getTranslation(), travelDir)
    );

    List<RotationTarget> holonomicRotations = List.of(
        new RotationTarget(0.0, startPose.getRotation()),
        new RotationTarget(1.0, targetRotation)
    );

    generatedPath = new PathPlannerPath(
        waypoints,
        holonomicRotations,
        new ArrayList<PointTowardsZone>(),
        new ArrayList<ConstraintsZone>(),
        new ArrayList<EventMarker>(),
        kConstraints,
        null,
        new GoalEndState(0.0, targetRotation),
        false
    );
    generatedPath.preventFlipping = true;

    allPathPoints = generatedPath.getAllPathPoints();
    if (allPathPoints == null || allPathPoints.isEmpty()) {
      return;
    }

    hadValidPath = true;
    logPathPoints();

    followCmd = AutoBuilder.followPath(generatedPath);
    followCmd.schedule();
  }

  @Override
  public void execute() {
    if (!hadValidPath || followCmd == null) return;
    if (!turnedGreen && !CommandScheduler.getInstance().isScheduled(followCmd)) {
      turnedGreen = true;
      new SetLedColor(0, 255, 0).schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setDriveToTagRunning(false);
    if (followCmd != null && CommandScheduler.getInstance().isScheduled(followCmd)) {
      followCmd.cancel();
    }
    drivetrain.setControl(stopRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    if (interrupted) {
      new DefaultLed().schedule();
    } else {
      new SetLedColor(0, 255, 0).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void logPathPoints() {
    double[] pathArray = new double[allPathPoints.size() * 2];
    for (int i = 0; i < allPathPoints.size(); i++) {
      PathPoint pt = allPathPoints.get(i);
      pathArray[i * 2] = pt.position.getX();
      pathArray[i * 2 + 1] = pt.position.getY();
    }
    SmartDashboard.putNumberArray("OnTheFly/PathPoints", pathArray);
  }
}