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

public class DriveToAprilTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric robotCentricDrive =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(
                com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage
            );

    // Simulation support
    private boolean useSimTestValues;
    private double simTagX;
    private double simTagY;
    private double simTagAngle;

    private static final double kSimDt = 0.02;

    // ===== OLD WORKING GAINS =====
    private static final double kP_translation = 1.5;
    private static final double kP_rotation = 2.0;

    private static final double kMaxSpeed = 2.0;      // m/s
    private static final double kMaxRotSpeed = 3.0;   // rad/s

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
        simTagAngle = 0.0;

        SmartDashboard.putBoolean(kAtTargetKey, false);
    }

    @Override
    public void execute() {
        double tagX, tagY, tagAngle;

        boolean inSim = Utils.isSimulation();

        if (inSim || useSimTestValues) {
            tagX = simTagX;
            tagY = simTagY;
            tagAngle = simTagAngle;
        } else {
            if (!LimelightHelpers.getTV(Constants.limelightName)) {
                drivetrain.setControl(
                    robotCentricDrive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                );
                SmartDashboard.putBoolean(kAtTargetKey, false);
                return;
            }

            Pose2d tagRelative = drivetrain.getAprilTagPose();
            tagX = tagRelative.getX();
            tagY = tagRelative.getY();
            tagAngle = tagRelative.getRotation().getRadians();
        }

        double vx = tagX * kP_translation;
        double vy = tagY * kP_translation;
        double vRot = 0.0;

        vx = clamp(vx, -kMaxSpeed, kMaxSpeed);
        vy = clamp(vy, -kMaxSpeed, kMaxSpeed);
        // vRot = clamp(vRot, -kMaxRotSpeed, kMaxRotSpeed);

        // Debug
        SmartDashboard.putNumber("Tag/X", tagX);
        SmartDashboard.putNumber("Tag/Y", tagY);
        SmartDashboard.putNumber("Tag/AngleDeg", Math.toDegrees(tagAngle));
        SmartDashboard.putNumber("Cmd/VX", vx);
        SmartDashboard.putNumber("Cmd/VY", vy);
        SmartDashboard.putNumber("Cmd/VRot", vRot);

        drivetrain.setControl(
            robotCentricDrive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vRot)
        );

        // ===== SIM UPDATE =====
        if (inSim || useSimTestValues) {
            simTagX -= vx * kSimDt;
            simTagY -= vy * kSimDt;
            simTagAngle -= vRot * kSimDt;

            if (simTagX < 0.1) simTagX = 0.1;

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
        return false; // button-held behavior preserved
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
