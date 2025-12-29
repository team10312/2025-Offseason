package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

/**
 * ORIGINAL SIMPLE APPROACH (restored):
 * - Constant forward speed (vx)
 * - "Left/right" correction using tag measurement (vy)
 * - No rotation (vRot = 0)
 *
 * Keeps:
 * - initialize/execute/end structure
 * - LED logic
 * - sim test values logic
 *
 * Finishes when distance to target < 1 meter.
 * Publishes "DriveToTag/AtTarget" boolean to SmartDashboard.
 */
public class DriveToAprilTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    // Robot-centric drive request
    private final SwerveRequest.RobotCentric robotCentricDrive =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(
                com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage
            );

    // Sim test values (these update each loop to simulate robot movement)
    private boolean useSimTestValues;
    private double simTagX;
    private double simTagY;
    private double simTagAngle;  // Simulated heading offset

    // Track last computed values for isFinished()
    private double lastTagX = Double.POSITIVE_INFINITY;
    private double lastTagY = Double.POSITIVE_INFINITY;
    private double lastAngle = Double.POSITIVE_INFINITY;
    
    // Minimum run time to prevent immediate ending
    private long startTimeMs = 0;
    private static final long kMinRunTimeMs = 500;  // Must run at least 0.5 seconds
    
    // Simulation timing
    private static final double kSimDt = 0.02;  // 20ms loop time

    // Proportional gains
    private static final double kP_translation = 0.8;
    private static final double kP_rotation = 2.0;  // Rotation gain
    private static final double kMaxRotRate = 2.0;  // Max rotation speed (rad/s)

    // Finish tolerances (tighter values to prevent early ending)
    private static final double kFinishDistanceX = 0.5;    // X distance tolerance (meters) - must be within 0.5m
    private static final double kFinishDistanceY = 0.05;   // Y offset tolerance (meters) - must be within 5cm
    private static final double kFinishAngleRad = Math.toRadians(3.0);  // Rotation tolerance - must be within 3 degrees

    // Dashboard key
    private static final String kAtTargetKey = "DriveToTag/AtTarget";

    public DriveToAprilTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        new AnimateLed(AnimationType.Rainbow).schedule();
        drivetrain.setDriveToTagRunning(true);

        useSimTestValues = drivetrain.getUseSimTestValues();
        simTagX = drivetrain.getSimTagX();
        simTagY = drivetrain.getSimTagY();
        simTagAngle = 0.0;  // Start with tag directly ahead in robot frame

        lastTagX = Double.POSITIVE_INFINITY;
        lastTagY = Double.POSITIVE_INFINITY;
        lastAngle = Double.POSITIVE_INFINITY;
        
        // Record start time
        startTimeMs = System.currentTimeMillis();

        // Start false on init
        SmartDashboard.putBoolean(kAtTargetKey, false);
    }

    @Override
    public void execute() {
        double tagX, tagY;

        boolean inSim = Utils.isSimulation();

        if (inSim || useSimTestValues) {
            // Use the simulated tag position (updated each loop)
            tagX = simTagX;
            tagY = simTagY;
        } else {
            if (!LimelightHelpers.getTV(Constants.limelightName)) {
                drivetrain.setControl(
                    robotCentricDrive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                );
                lastTagX = Double.POSITIVE_INFINITY;
                lastTagY = Double.POSITIVE_INFINITY;
                lastAngle = Double.POSITIVE_INFINITY;
                SmartDashboard.putBoolean(kAtTargetKey, false);
                return;
            }

            Pose2d tagRelative = drivetrain.getAprilTagPose();
            tagX = tagRelative.getX();
            tagY = tagRelative.getY();
        }

        // Distance to tag (used for speed limiting)
        double distance = Math.sqrt(tagX * tagX + tagY * tagY);
        
        // Calculate angle to tag (when 0, tag is directly in front)
        // Positive angleToTag means tag is to the left, negative means to the right
        double angleToTag = Math.atan2(tagY, tagX);
        
        // Store values for isFinished()
        lastTagX = tagX;
        lastTagY = tagY;
        lastAngle = angleToTag;

        // Check if at target (all three conditions must be met)
        // X must be positive (tag in front) AND within tolerance
        boolean xOk = tagX > 0 && tagX < kFinishDistanceX;
        boolean yOk = Math.abs(tagY) < kFinishDistanceY;
        boolean angleOk = Math.abs(angleToTag) < kFinishAngleRad;
        boolean atTarget = xOk && yOk && angleOk;
        
        // Debug output for finish conditions
        SmartDashboard.putBoolean(kAtTargetKey, atTarget);
        SmartDashboard.putBoolean("DriveToTag/X_OK", xOk);
        SmartDashboard.putBoolean("DriveToTag/Y_OK", yOk);
        SmartDashboard.putBoolean("DriveToTag/Angle_OK", angleOk);
        SmartDashboard.putNumber("DriveToTag/LastTagX", tagX);
        SmartDashboard.putNumber("DriveToTag/LastTagY", tagY);
        SmartDashboard.putNumber("DriveToTag/LastAngleDeg", Math.toDegrees(angleToTag));
        
        // ===== CONTROL =====
        // Rotation: turn to face the tag
        // If tag is to the left (angleToTag > 0), rotate left (positive vRot)
        double vRot = angleToTag * kP_rotation;  // Flipped sign to rotate TOWARD the tag
        vRot = clamp(vRot, -kMaxRotRate, kMaxRotRate);
        
        // Forward speed: reduce when not aligned (creates curved path)
        // cos(angleToTag) = 1 when aligned, 0 when perpendicular, -1 when facing away
        double alignmentFactor = Math.max(0, Math.cos(angleToTag));  // 0 to 1
        double baseSpeed = 0.8;  // Base forward speed
        double vx = baseSpeed * alignmentFactor;  // Slow down when not facing tag
        
        // Strafe: move toward the tag to center on it
        // Limelight Y axis appears inverted, so negate tagY
        double vy = -tagY * kP_translation * 0.5;
        
        // Speed limiting based on distance
        double maxSpeed;
        if (distance < 0.3) {
            maxSpeed = 0.3;  // Slow final approach
        } else if (distance < 0.5) {
            maxSpeed = 0.5;
        } else if (distance < 1.0) {
            maxSpeed = 1.0;
        } else {
            maxSpeed = 2.0;
        }

        vx = clamp(vx, 0, maxSpeed);  // Only forward, no backward
        vy = clamp(vy, -maxSpeed, maxSpeed);

        // Debug output
        SmartDashboard.putNumber("Tag/X", tagX);
        SmartDashboard.putNumber("Tag/Y", tagY);
        SmartDashboard.putNumber("Tag/Distance", distance);
        SmartDashboard.putNumber("Tag/AngleDeg", Math.toDegrees(angleToTag));
        SmartDashboard.putNumber("Cmd/VX", vx);
        SmartDashboard.putNumber("Cmd/VY", vy);
        SmartDashboard.putNumber("Cmd/VRot", vRot);

        drivetrain.setControl(
            robotCentricDrive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vRot)
        );
        
        // Update simulated tag position based on robot movement (only in sim)
        if (inSim || useSimTestValues) {
            // Simple simulation: update tag position based on robot velocity
            // Robot moving forward decreases tag X (getting closer)
            simTagX -= vx * kSimDt;
            // Robot strafing left decreases tag Y (aligning)
            simTagY -= vy * kSimDt;
            
            // Clamp X to prevent going behind robot
            if (simTagX < 0.1) simTagX = 0.1;
            
            // Debug: show simulated values
            SmartDashboard.putNumber("Sim/TagX", simTagX);
            SmartDashboard.putNumber("Sim/TagY", simTagY);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDriveToTagRunning(false);

        drivetrain.setControl(
            robotCentricDrive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    if (!interrupted) {
        new SetLedColor(0, 255, 0).schedule();
        SmartDashboard.putBoolean(kAtTargetKey, true);
    } else {
        new DefaultLed().schedule();
        SmartDashboard.putBoolean(kAtTargetKey, false);
    }
}


    @Override
    public boolean isFinished() {
        // DISABLED: Command runs until button released
        // This lets you test the movement without early ending
        // TODO: Re-enable when movement is correct
        return false;
        
        /*
        // Don't finish before minimum run time
        long elapsed = System.currentTimeMillis() - startTimeMs;
        if (elapsed < kMinRunTimeMs) {
            return false;
        }
        
        // All three conditions must be met
        // X must be positive (tag in front) AND within tolerance
        // Also require X > 0.1 to filter out invalid (0,0) readings
        boolean xOk = lastTagX > 0.1 && lastTagX < kFinishDistanceX;
        boolean yOk = Math.abs(lastTagY) < kFinishDistanceY;
        boolean angleOk = Math.abs(lastAngle) < kFinishAngleRad;
        return xOk && yOk && angleOk;
        */
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
