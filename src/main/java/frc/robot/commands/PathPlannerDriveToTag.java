package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

/**
 * PathPlannerDriveToTag - Advanced AprilTag alignment using PathPlanner.
 * 
 * Based on best practices from top FRC teams (254, 401, etc.):
 * - Uses PathPlanner for smooth, obstacle-aware paths
 * - Works in both field-relative (on FRC field) and robot-relative (garage) modes
 * - Handles tag loss gracefully using odometry
 * - Provides smooth motion with proper velocity/acceleration profiles
 * 
 * Two modes:
 * 1. Field-relative: On actual FRC field with known AprilTag positions
 *    - Uses pathfindToPose() with field coordinates
 *    - More robust, handles obstacles
 * 2. Robot-relative: In garage/testing with random AprilTag
 *    - Uses waypoint-based path generation
 *    - Works without field layout
 */
public class PathPlannerDriveToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private Command activePathCommand = null;
    private boolean shouldFinishImmediately = false;

    // Finish tolerances
    private static final double kDistanceTolerance = 0.3;  // 30cm
    private static final double kAngleToleranceRad = Units.degreesToRadians(2.0);  // 2 degrees

    public PathPlannerDriveToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset state
        activePathCommand = null;
        shouldFinishImmediately = false;
        
        // Turn on rainbow LEDs
        new AnimateLed(AnimationType.Rainbow).schedule();
        drivetrain.setDriveToTagRunning(true);
        
        SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Initializing...");
        SmartDashboard.putBoolean("PathPlannerDriveToTag/Running", true);
        
        // Check if tag is visible
        if (!LimelightHelpers.getTV(Constants.limelightName)) {
            SmartDashboard.putString("PathPlannerDriveToTag/Mode", "No Tag Visible");
            SmartDashboard.putBoolean("PathPlannerDriveToTag/Running", false);
            shouldFinishImmediately = true;
            return;
        }
        
        // Get tag position relative to robot
        Pose2d tagRelative = drivetrain.getAprilTagPose();
        double tagX = tagRelative.getX();
        double tagY = tagRelative.getY();
        double distance = Math.sqrt(tagX * tagX + tagY * tagY);
        
        // Check if we're already close enough
        if (distance < kDistanceTolerance) {
            SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Already at Target");
            SmartDashboard.putBoolean("PathPlannerDriveToTag/Running", false);
            shouldFinishImmediately = true;
            return;
        }
        
        // Try to get field-relative robot pose
        Pose2d robotFieldPose = drivetrain.getLLPose();
        boolean hasFieldPose = isValidFieldPose(robotFieldPose);
        
        if (hasFieldPose) {
            // MODE 1: Field-relative pathfinding (on actual FRC field)
            SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Field-Relative");
            
            // Calculate target pose in field coordinates
            // Transform robot-relative tag pose to field-relative
            Pose2d targetFieldPose = robotFieldPose.transformBy(
        new Transform2d(
            tagX,
            tagY,
            tagRelative.getRotation()
        )
        );

            
            // Face the tag at the end
            Rotation2d targetRotation = Rotation2d.fromRadians(
                Math.atan2(tagY, tagX)
            );
            targetFieldPose = new Pose2d(targetFieldPose.getTranslation(), targetRotation);
            
            // Reset odometry to vision pose for accuracy
            drivetrain.resetOdometry(robotFieldPose);
            
            // Use PathPlanner's pathfinding
            activePathCommand = AutoBuilder.pathfindToPose(
                targetFieldPose,
                Constants.constraints,
                0.0
            )
            .until(() -> isAtTarget())
            .finallyDo(() -> {
                drivetrain.setDriveToTagRunning(false);
                SmartDashboard.putBoolean("PathPlannerDriveToTag/Running", false);
            });
            
        } else {
            // MODE 2: Robot-relative waypoint path (garage/testing)
            SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Robot-Relative");
            
            // Create path from current position (0,0) to tag position in robot space
            Pose2d startPose = new Pose2d(0, 0, drivetrain.getEstimatedPose().getRotation());
            
            // Calculate desired end rotation to face the tag
            Rotation2d endRotation = Rotation2d.fromRadians(Math.atan2(tagY, tagX));
            Pose2d endPose = new Pose2d(tagX, tagY, endRotation);
            
            // Create waypoints for smooth path
            java.util.List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPose,
                endPose
            );
            
            // Create path with constraints
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                Constants.constraints,
                null, // global constraints already in constraints object
                new GoalEndState(0.0, endRotation) // stop at end, facing tag
            );
            
            // Log path for visualization
            PathPlannerLogging.logActivePath(path);
            
            // Reset odometry to origin for robot-relative path
            drivetrain.resetOdometry(new Pose2d(0, 0, drivetrain.getEstimatedPose().getRotation()));
            
            // Follow the path
            activePathCommand = AutoBuilder.followPath(path)
                .until(() -> isAtTarget())
                .finallyDo(() -> {
                    drivetrain.setDriveToTagRunning(false);
                    SmartDashboard.putBoolean("PathPlannerDriveToTag/Running", false);
                });
        }
        
        // Schedule the path command
        if (activePathCommand != null) {
            activePathCommand.schedule();
        }
    }

    @Override
    public void execute() {
        // The actual path following is handled by the scheduled command
        // We just monitor and update SmartDashboard here
        if (activePathCommand != null && !activePathCommand.isScheduled()) {
            // Path command finished
            SmartDashboard.putBoolean("PathPlannerDriveToTag/PathComplete", true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDriveToTagRunning(false);
        SmartDashboard.putBoolean("PathPlannerDriveToTag/Running", false);
        
        // Cancel the path command if it's still running
        if (activePathCommand != null && activePathCommand.isScheduled()) {
            activePathCommand.cancel();
        }
        
        // Stop the robot
        drivetrain.stop();
        
        if (!interrupted) {
            // Success - green LED
            new SetLedColor(0, 255, 0).schedule();
            SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Completed");
        } else {
            // Interrupted - default LED
            new DefaultLed().schedule();
            SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        // If we should finish immediately (no tag or already at target), finish now
        if (shouldFinishImmediately) {
            return true;
        }
        
        // Command finishes when the path command finishes or we're at target
        if (activePathCommand != null && !activePathCommand.isScheduled()) {
            return true;
        }
        return isAtTarget();
    }

    /**
     * Check if robot is at the target AprilTag.
     * Uses both distance and angle tolerance for accurate completion.
     */
    private boolean isAtTarget() {
        if (!LimelightHelpers.getTV(Constants.limelightName)) {
            return false; // No tag visible, can't be at target
        }
        
        Pose2d tagRelative = drivetrain.getAprilTagPose();
        double tagX = tagRelative.getX();
        double tagY = tagRelative.getY();
        double distance = Math.sqrt(tagX * tagX + tagY * tagY);
        
        // Check distance tolerance
        boolean distanceOk = distance < kDistanceTolerance;
        
        // Check angle tolerance (tag should be roughly in front)
        double angleToTag = Math.atan2(tagY, tagX);
        boolean angleOk = Math.abs(angleToTag) < kAngleToleranceRad;
        
        // Update SmartDashboard
        SmartDashboard.putBoolean("PathPlannerDriveToTag/AtTarget", distanceOk && angleOk);
        SmartDashboard.putNumber("PathPlannerDriveToTag/Distance", distance);
        SmartDashboard.putNumber("PathPlannerDriveToTag/AngleDeg", Units.radiansToDegrees(angleToTag));
        
        return distanceOk && angleOk;
    }

    /**
     * Check if a field pose is valid (not just zeros or clearly invalid).
     * This determines whether we're on an actual FRC field or in garage/testing.
     */
    private boolean isValidFieldPose(Pose2d pose) {
        // Field coordinates should be within reasonable bounds
        // FRC field is roughly 16.5m x 8m, so valid poses should be in that range
        double x = pose.getX();
        double y = pose.getY();
        
        // Check if pose is clearly invalid (all zeros or way out of bounds)
        // Also check if it's a default/invalid pose from Limelight
        boolean isZero = Math.abs(x) < 0.01 && Math.abs(y) < 0.01;
        boolean inBounds = x > -2 && x < 18 && y > -2 && y < 10; // Slightly larger than field
        
        return !isZero && inBounds;
    }
}

