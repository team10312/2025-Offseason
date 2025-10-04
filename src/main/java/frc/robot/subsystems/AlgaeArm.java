// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.Constants;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeArm extends SubsystemBase {
    private static AlgaeArm mInstance;

    private static AlgaeArm getInstance(){
        if(mInstance == null){
            mInstance = new AlgaeArm();
        }
        return mInstance;
    }

    private final TalonFX mArm;
    final MotionMagicVoltage mMotionMagicRequest = new MotionMagicVoltage(0);

    public AlgaeArm() {

    mArm = new TalonFX(34, "rio");


  }

public double rotationsToDegrees(double rotations){
    //should add home pos
    return (rotations / Constants.mArmGearRatio * 360);
}

public double degreesToRotations(double degrees){
    return ((degrees - 0) / 360) * Constants.mArmGearRatio;
}

public double getDegrees(){
    return rotationsToDegrees(mArm.getPosition().getValueAsDouble());
}

public void setDegrees(double degrees){
    mArm.setControl(mMotionMagicRequest.withPosition(degreesToRotations(degrees)));
    // mArm.setPosition(degrees);
}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Arm Degrees", getDegrees());
  }


}