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

  // SIM: fixed AprilTag pose in field (constant)
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

    // Reset Dashboard logs
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
    Pose2d tagFieldPose;

    // -------------------------------------------------------------------------
    // STEP 1: CALCULATE TAG FIELD POSE
    // -------------------------------------------------------------------------
    if (inSimulation) {
      // SIMULATION: Use fixed constant tag
      tagFieldPose = kSimTagFieldPose;
    } else {
      // REAL ROBOT: Use Limelight
      if (!LimelightHelpers.getTV(Constants.limelightName)) {
        System.out.println("OnTheFly: No Tag Visible");
        return;
      }

      // 1. Get Robot-Relative Pose from Limelight
      // (Limelight Robot Space: X=Forward, Y=Right, Z=Up usually)
      Pose2d tagRelative = drivetrain.getAprilTagPose();

      // 2. COORDINATE CORRECTION (Crucial Step)
      // WPILib Robot Space: X=Forward, Y=Left
      // We must invert Y to convert "Right-Positive" to "Left-Positive"
      Translation2d correctedTranslation = new Translation2d(
          tagRelative.getX(),
          -tagRelative.getY() // <--- INVERT Y HERE
      );

      // 3. Transform StartPose to FieldPose
      // We assume rotation is 0 for the transform because we only care about position X/Y here
      Transform2d robotToTag = new Transform2d(correctedTranslation, new Rotation2d());
      Pose2d rawTagPos = startPose.transformBy(robotToTag);

      // 4. Force Tag Orientation to "Face" the Robot
      // This ensures we always approach from the front, regardless of how the tag is angled on the wall
      Translation2d tagToRobot = startPose.getTranslation().minus(rawTagPos.getTranslation());
      
      // If we are practically on top of the tag, keep current rotation to avoid spinning
      Rotation2d tagFacing = (tagToRobot.getNorm() < 0.1) 
          ? startPose.getRotation().plus(Rotation2d.k180deg) 
          : tagToRobot.getAngle();

      tagFieldPose = new Pose2d(rawTagPos.getTranslation(), tagFacing);
    }

    // -------------------------------------------------------------------------
    // STEP 2: CALCULATE TARGET POSE
    // -------------------------------------------------------------------------
    // Calculate the vector pointing FROM Tag TO Robot
    // (We forced tagFieldPose rotation to point at robot in REAL block above,
    //  and in SIM block the math handles it similarly).
    Translation2d approachVector = new Translation2d(kStandoffDistance, 0).rotateBy(tagFieldPose.getRotation());
    
    // Target is Tag Location + Offset vector
    Translation2d targetTranslation = tagFieldPose.getTranslation().plus(approachVector);

    // Robot should face OPPOSITE to the tag's facing vector (Look At Tag)
    targetRotation = tagFieldPose.getRotation().plus(Rotation2d.k180deg);

    targetPose = new Pose2d(targetTranslation, targetRotation);

    // LOGGING
    SmartDashboard.putNumberArray("OnTheFly/TargetPose", new double[] {
        targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians()
    });

    // -------------------------------------------------------------------------
    // STEP 3: GENERATE PATH
    // -------------------------------------------------------------------------
    double distToTarget = startPose.getTranslation().getDistance(targetPose.getTranslation());
    
    // Safety: If we are already at the target (< 10cm), do not move, just finish.
    if (distToTarget < 0.1) {
        System.out.println("OnTheFly: Already at target!");
        turnedGreen = true;
        new SetLedColor(0, 255, 0).schedule();
        return; 
    }

    // Standard travel direction calculation
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

    // Check if path following command has finished
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

    // Stop the robot
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