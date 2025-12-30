package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import org.littletonrobotics.junction.Logger;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private boolean m_isPathFollowing = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    //Boolean Checks
    public boolean pathScheduled = false;
    private boolean driveToTagRunning = false;

    // Simulation test values for AprilTag (can be adjusted via SmartDashboard)
    private boolean useSimTestValues = false;  // Toggle to use test values even in real mode
    private double simTagX = 2.0;  // Test tag X position (meters, forward)
    private double simTagY = 1.0;  // Test tag Y position (meters, left)
    private double simTagYawRad = Units.degreesToRadians(20.0);
    private Pose2d simTagPose =
    new Pose2d(
        simTagX,
        simTagY,
        new Rotation2d(simTagYawRad)
    );
    private static final AprilTagFieldLayout kTagLayout =
    AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
    // Fixed simulated AprilTag pose on the field



    // Getters and setters for DriveToAprilTag command
    public void setDriveToTagRunning(boolean running) {
        driveToTagRunning = running;
    }

    public boolean getUseSimTestValues() {
        return useSimTestValues;
    }

    public double getSimTagX() {
        return simTagX;
    }

    public double getSimTagY() {
        return simTagY;
    }

    public double getSimTagYawRad() {
        return simTagYawRad;
    }    

    public Pose2d getSimAprilTagPose() {
        return simTagPose;
    }    

    public Pose2d getSimAprilTagRobotPose() {
        return simTagPose.relativeTo(getEstimatedPose());
    }    

    public Pose2d getRobotFieldRelativePose() {
        // 1. Get current alliance (Default to Blue if not connected)
        boolean isRed = !(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    
        LimelightHelpers.PoseEstimate estimate = isRed 
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-low")
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-low");
    

        if (estimate == null || estimate.tagCount == 0) {
            return getEstimatedPose();
        }
    
        return new Pose2d(
            estimate.pose.getX(), 
            estimate.pose.getY(), 
            estimate.pose.getRotation()
        );
    }

    public Pose2d getAprilTagFieldRelativePose(){
        int tagID = (int) LimelightHelpers.getFiducialID("limelight-low");

        if (Utils.isSimulation()){
            tagID = 7;
        }

        if (tagID > 0) {
            return kTagLayout.getTagPose(tagID).orElse(new Pose3d()).toPose2d();        
        }
        return new Pose2d();
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *        unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *        unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *        in the form [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation
     *        in the form [x, y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getState().Speeds; // Phoenix 6 provides measured robot-relative speeds
    }

    /** Reset odometry to a given field pose (seed CTRE odometry). */
    public void resetOdometry(Pose2d pose) {
        this.resetPose(pose); // CTRE Tuner method
    }

    public Pose2d getEstimatedPose(){
        return this.getState().Pose;
    }
    
    public Pose2d getLLPose(){
        return new Pose2d(LimelightHelpers.getBotPose2d(Constants.limelightName).getX(), LimelightHelpers.getBotPose2d(Constants.limelightName).getY(), LimelightHelpers.getBotPose3d(Constants.limelightName).getRotation().toRotation2d());
    }

    public Pose2d getAprilTagPose(){
        return new Pose2d(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getX(), LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getY(), LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getRotation().toRotation2d());
    }
    
    // Apply the chassis speeds
    private final SwerveRequest.ApplyRobotSpeeds ppApplySpeeds = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    
    // Robot-centric drive request (matches FieldCentric config)
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public void driveRobotRelative(ChassisSpeeds speeds){
        this.setControl(ppApplySpeeds.withSpeeds(speeds));
    }

    private void configurePathPlanner(){
        RobotConfig ppRobotConfig;
        try{
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Pathplanner Config Missing");
        }

        PPHolonomicDriveController ppController = new PPHolonomicDriveController(
            new PIDConstants(Constants.TRANS_KP, Constants.TRANS_KI, Constants.TRANS_KD),
            new PIDConstants(Constants.ROT_KP, Constants.ROT_KI, Constants.ROT_KD)
        );

        AutoBuilder.configure(
            this::getEstimatedPose, // pose supplier
            this::resetOdometry, // reset odometry
            this::getRobotRelativeSpeeds, // robot-relative speeds supplier
            this::driveRobotRelative, // robot-relative drive consumer
            ppController, // holonomic follower controller
            ppRobotConfig, // robot config from GUI
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, // flip paths on red if needed
            this // this subsystem owns the requirement
        );

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    }

    public void stop(){
        this.setControl(ppApplySpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }



    public boolean isAtTarget() {
        // No tag visible → not at target
        if (!LimelightHelpers.getTV(Constants.limelightName)) return false;
    
        Pose2d tagRelative = getAprilTagPose();
    
        double x = tagRelative.getX();
        double y = tagRelative.getY();
    
        // Distance check (within 1 meter)
        double distSq = x * x + y * y;
        double distTol = 1.0; // meters
    
        // Rotation tolerance: ±2 degrees
        double angleToTagRad = Math.atan2(y, x);
        double rotTolRad = Units.degreesToRadians(2.0);
    
        return distSq < distTol * distTol
            && Math.abs(angleToTagRad) < rotTolRad;
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
public void periodic() {
    // Apply operator perspective safely
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        });
    }

    // ===================== LIMELIGHT POSES =====================

    // SmartDashboard.putNumber(null, kNumConfigAttempts);
    SmartDashboard.putNumberArray("Poses/Robot Field Relative", 
        new double[]{
            getRobotFieldRelativePose().getX(), 
            getRobotFieldRelativePose().getY(), 
            getRobotFieldRelativePose().getRotation().getDegrees()
        }
    );
    SmartDashboard.putNumberArray("Poses/AprilTag Field Relative", 
        new double[]{
            getAprilTagFieldRelativePose().getX(), 
            getAprilTagFieldRelativePose().getY(), 
            getAprilTagFieldRelativePose().getRotation().getDegrees()
        }
    );

    



    simTagX = SmartDashboard.getNumber("SimTag/X_m", simTagX);
    simTagY = SmartDashboard.getNumber("SimTag/Y_m", simTagY);
    simTagYawRad = Units.degreesToRadians(
        SmartDashboard.getNumber(
            "SimTag/Yaw_deg",
            Units.radiansToDegrees(simTagYawRad)
        )
    );

    simTagPose = new Pose2d(
        simTagX,
        simTagY,
        new Rotation2d(simTagYawRad)
    );

    Logger.recordOutput("Sim/AprilTagPose", getSimAprilTagRobotPose());
}


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *        in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}