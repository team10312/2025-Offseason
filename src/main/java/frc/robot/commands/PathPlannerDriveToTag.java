package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

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
 * PathPlannerDriveToTag - AprilTag alignment using PathPlanner with "Local Virtual Field" approach.
 * 
 * Key principle: PathPlanner only requires a CONSISTENT coordinate frame, not a real FRC field.
 * 
 * How it works:
 * 1. Origin is established once at first robot enable (0,0,0 = starting position)
 * 2. Robot pose is tracked via odometry in this virtual field
 * 3. AprilTag relative pose is converted to virtual-field coordinates
 * 4. PathPlanner navigates to a goal pose in virtual-field coordinates
 * 
 * This approach works identically in:
 * - Garage/shop testing (with any AprilTag)
 * - Competition field (same code, same math)
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
        
        // Get current robot pose in our virtual field coordinate system
        // (Origin was established once at first enable)
        Pose2d robotFieldPose = drivetrain.getEstimatedPose();
        
        SmartDashboard.putString("PathPlannerDriveToTag/Mode", "Virtual-Field");
        
        // Convert tag-relative pose to virtual-field pose
        // This is the key step: robotPose ⊕ tagRelativePose → tagFieldPose
        Pose2d tagFieldPose = robotFieldPose.transformBy(
            new Transform2d(tagX, tagY, tagRelative.getRotation())
        );
        
        // Calculate target pose: offset from tag, facing toward it
        // Stand 0.5m in front of the tag, facing it
        double standoffDistance = 0.5;
        Pose2d targetFieldPose = tagFieldPose.transformBy(
            new Transform2d(-standoffDistance, 0.0, Rotation2d.k180deg)
        );
        
        SmartDashboard.putNumberArray("PathPlannerDriveToTag/RobotPose", 
            new double[]{robotFieldPose.getX(), robotFieldPose.getY(), robotFieldPose.getRotation().getDegrees()});
        SmartDashboard.putNumberArray("PathPlannerDriveToTag/TagFieldPose", 
            new double[]{tagFieldPose.getX(), tagFieldPose.getY(), tagFieldPose.getRotation().getDegrees()});
        SmartDashboard.putNumberArray("PathPlannerDriveToTag/TargetPose", 
            new double[]{targetFieldPose.getX(), targetFieldPose.getY(), targetFieldPose.getRotation().getDegrees()});
        
        // Use PathPlanner's pathfinding (works in any consistent coordinate frame)
        // NO odometry reset here - we use the existing virtual field origin
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

}

