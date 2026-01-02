package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class custom extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private Command pathCommand = null;
    private boolean finished = false;

    public custom(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
    }

    @Override
    public void initialize() {
        pathCommand = null;
        finished = false;

        DriverStation.reportWarning("[custom] initialize() starting", false);

        boolean tv = LimelightHelpers.getTV(Constants.limelightName);
        SmartDashboard.putBoolean("custom/tv", tv);

        if (!tv) {
            DriverStation.reportWarning("[custom] No tag visible - finishing", false);
            finished = true;
            return;
        }

        // ------------------------------------------------------------
        // STEP 1: CURRENT ROBOT POSE (ODOMETRY)
        // ------------------------------------------------------------
        Pose2d currentPose = drivetrain.getEstimatedPose();

        // ------------------------------------------------------------
        // STEP 2: ROBOT POSE RELATIVE TO TAG (THE IMPORTANT FIX)
        // ------------------------------------------------------------
        Pose3d robotInTagSpace =
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.limelightName);

        double x = robotInTagSpace.getX(); // forward/back
        double y = robotInTagSpace.getY(); // left/right

        double distance = Math.hypot(x, y);
        SmartDashboard.putNumber("custom/tagDistanceMeters", distance);

        // If we’re already basically there, don’t move
        if (distance < 0.15) {
            DriverStation.reportWarning("[custom] Already at target", false);
            finished = true;
            return;
        }

        // ------------------------------------------------------------
        // STEP 3: TRANSLATION NEEDED TO REACH TAG (ROBOT RELATIVE)
        // NEGATIVE because we want to drive TO the tag
        // ------------------------------------------------------------
        Translation2d deltaRobotRelative =
            new Translation2d(-x, -y);

        // ------------------------------------------------------------
        // STEP 4: CONVERT TO FIELD RELATIVE TARGET POSE
        // ------------------------------------------------------------
        Pose2d targetPose = currentPose.transformBy(
            new Transform2d(deltaRobotRelative, new Rotation2d())
        );

        // ------------------------------------------------------------
        // STEP 5: ROTATION — FACE THE TAG
        // ------------------------------------------------------------
        Rotation2d goalRotation =
            deltaRobotRelative.getAngle().plus(currentPose.getRotation());

        Pose2d finalTargetPose = new Pose2d(
            targetPose.getTranslation(),
            goalRotation
        );

        SmartDashboard.putNumberArray(
            "custom/finalTargetPose",
            new double[] {
                finalTargetPose.getX(),
                finalTargetPose.getY(),
                finalTargetPose.getRotation().getRadians()
            }
        );

        // ------------------------------------------------------------
        // STEP 6: PATHPLANNER PATHFIND
        // ------------------------------------------------------------
        pathCommand = AutoBuilder.pathfindToPose(
            finalTargetPose,
            Constants.constraints,
            0.0
        );

        pathCommand.schedule();
        DriverStation.reportWarning("[custom] pathCommand scheduled", false);
    }

    @Override
    public void execute() {
        // PathPlanner command runs independently
    }

    @Override
    public void end(boolean interrupted) {
        DriverStation.reportWarning(
            "[custom] end() interrupted=" + interrupted,
            false
        );
    }

    @Override
    public boolean isFinished() {
        if (finished) {
            DriverStation.reportWarning("[custom] isFinished: finished flag", false);
            return true;
        }

        if (pathCommand == null) {
            DriverStation.reportWarning("[custom] isFinished: pathCommand null", false);
            return true;
        }

        if (!pathCommand.isScheduled()) {
            DriverStation.reportWarning("[custom] isFinished: path complete", false);
            return true;
        }

        return false;
    }
}
