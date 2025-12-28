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

    // Sim test values
    private boolean useSimTestValues;
    private double simTagX;
    private double simTagY;

    // Track last computed distance for isFinished()
    private double lastDistance = Double.POSITIVE_INFINITY;

    // Original proportional gain
    private static final double kP_translation = 0.8;

    // Finish tolerance (meters)
    private static final double kFinishDistanceM = 1.0;

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

        lastDistance = Double.POSITIVE_INFINITY;

        // Start false on init
        SmartDashboard.putBoolean(kAtTargetKey, false);
    }

    @Override
    public void execute() {
        double tagX, tagY;

        boolean inSim = Utils.isSimulation();

        if (inSim || useSimTestValues) {
            // Pull latest values so you can change them live via SmartDashboard in drivetrain.periodic()
            tagX = drivetrain.getSimTagX();
            tagY = drivetrain.getSimTagY();
        } else {
            if (!LimelightHelpers.getTV(Constants.limelightName)) {
                drivetrain.setControl(
                    robotCentricDrive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                );
                lastDistance = Double.POSITIVE_INFINITY;
                SmartDashboard.putBoolean(kAtTargetKey, false);
                return;
            }

            Pose2d tagRelative = drivetrain.getAprilTagPose();
            tagX = tagRelative.getX();
            tagY = tagRelative.getY();
        }

        // Distance to tag (used for speed limiting + isFinished)
        double distance = Math.sqrt(tagX * tagX + tagY * tagY);
        lastDistance = distance;

        boolean atTarget = distance < kFinishDistanceM;
        SmartDashboard.putBoolean(kAtTargetKey, atTarget);

        // ===== ORIGINAL CONTROL =====
        double vx = 0.5;                    // constant forward speed
        double vy = -tagX * kP_translation; // original left/right mapping (restored exactly)
        double vRot = 0.0;                  // no rotation

        // ===== SPEED LIMITING (original) =====
        double maxSpeed;
        if (distance < 0.5) {
            maxSpeed = 0.5;
        } else if (distance < 1.0) {
            maxSpeed = 1.0;
        } else {
            maxSpeed = 2.0;
        }

        vx = clamp(vx, -maxSpeed, maxSpeed);
        vy = clamp(vy, -maxSpeed, maxSpeed);

        // Debug output (original-style)
        SmartDashboard.putNumber("Tag/X", tagX);
        SmartDashboard.putNumber("Tag/Y", tagY);
        SmartDashboard.putNumber("Tag/Distance", distance);
        SmartDashboard.putNumber("Cmd/VX", vx);
        SmartDashboard.putNumber("Cmd/VY", vy);

        drivetrain.setControl(
            robotCentricDrive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vRot)
        );
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

        // If we ended early (interrupted), mark false. If we ended naturally, leave whatever execute() last set.
        if (interrupted) {
            SmartDashboard.putBoolean(kAtTargetKey, false);
        }

        new StopLed().schedule();
    }

    @Override
    public boolean isFinished() {
        return lastDistance < kFinishDistanceM;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
