// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leds extends SubsystemBase {
    private static Leds mInstance;

    public static Leds getInstance(){
        if(mInstance == null){
            mInstance = new Leds();
        }
        return mInstance;
    }

    private final CANdle m_candle;
    private final int ledCount = 65;

    private enum LedMode { Unknown, SolidColor, Animation }

    private LedMode m_ledMode = LedMode.Unknown;
    private AnimationType m_currentAnimation = null;
    private boolean m_hasColor = false;
    private int m_r = 0;
    private int m_g = 0;
    private int m_b = 0;

    public String getLedState() {
        LedMode mode = m_ledMode;
        AnimationType anim = m_currentAnimation;

        if (mode == LedMode.Animation) {
            if (m_hasColor) {
                return "Animation=" + (anim == null ? "null" : anim.name())
                        + " RGB=(" + m_r + "," + m_g + "," + m_b + ")";
            }
            return "Animation=" + (anim == null ? "null" : anim.name()) + " RGB=(none)";
        }

        if (mode == LedMode.SolidColor) {
            return "SolidColor RGB=(" + m_r + "," + m_g + "," + m_b + ")";
        }

        return "Unknown";
    }

    public boolean isAnimationRunning() {
        return m_ledMode == LedMode.Animation;
    }

    public AnimationType getCurrentAnimation() {
        return m_currentAnimation;
    }

    private void setTrackedSolid(int r, int g, int b) {
        m_ledMode = LedMode.SolidColor;
        m_currentAnimation = AnimationType.None;
        m_hasColor = true;
        m_r = r;
        m_g = g;
        m_b = b;
    }

    private void setTrackedAnimation(AnimationType type) {
        m_ledMode = LedMode.Animation;
        m_currentAnimation = type;
    }

    private void clearTrackedColor() {
        m_hasColor = false;
        m_r = 0;
        m_g = 0;
        m_b = 0;
    }

    public Leds() {

        m_candle = new CANdle(44, "rio");
        var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        cfg.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
        cfg.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        m_candle.getConfigurator().apply(cfg);
  }

  public enum AnimationType {
    None(false),
    ColorFlow(true),
    Fire(false),
    Larson(true),
    Rainbow(false),
    RgbFade(false),
    SingleFade(true),
    Strobe(true),
    Twinkle(true),
    TwinkleOff(true);

    public final boolean requiresColor;

    AnimationType(boolean requiresColor) {
        this.requiresColor = requiresColor;
    }
}

  private void setAnimationInternal(AnimationType type, RGBWColor color) {
    m_candle.setControl(new EmptyAnimation(0));

    setTrackedAnimation(type);
    if (color == null) {
        clearTrackedColor();
    }

    switch (type) {

        case Fire:
            m_candle.setControl(
                new FireAnimation(0, ledCount)
                    .withSlot(0)
                    .withCooling(0.4)
                    .withSparking(0.5)
            );
            break;

        case Rainbow:
            m_candle.setControl(
                new RainbowAnimation(0, ledCount)
                    .withSlot(0)
            );
            break;

        case Larson:
            m_candle.setControl(
                new LarsonAnimation(0, ledCount)
                    .withSlot(0)
                    .withColor(color)
            );
            break;

        case ColorFlow:
            m_candle.setControl(
                new ColorFlowAnimation(0, ledCount)
                    .withSlot(0)
                    .withColor(color)
            );
            break;

        case Twinkle:
            m_candle.setControl(
                new TwinkleAnimation(0, ledCount)
                    .withSlot(0)
                    .withColor(color)
            );
            break;

        case None:
            m_candle.setControl(
                new SolidColor(0, ledCount)
                    .withColor(new RGBWColor(0, 0, 0))
            );
            break;

        default:
            break;
    }
  }

  public void setAnimation(AnimationType type){
    if (type.requiresColor) {
      throw new IllegalArgumentException(
          type + " requires r, g, b"
      );
  }
    setAnimationInternal(type, null);
  }

  public void setAnimation(AnimationType type, int r, int g, int b){
    m_hasColor = true;
    m_r = r;
    m_g = g;
    m_b = b;

    setAnimationInternal(type, new RGBWColor(r, g, b));
  }

  public void setColor(int r, int g, int b){
    setAnimation(AnimationType.None);
    m_candle.setControl(new SolidColor(0, ledCount).withColor(new RGBWColor(r, g, b)));
    setTrackedSolid(r, g, b);
  }

  public void off(){
    m_candle.setControl(new SolidColor(0, ledCount).withColor(new RGBWColor(0, 0, 0, 0)));
    setAnimation(AnimationType.None);
    setTrackedSolid(0, 0, 0);
  }
  
  @Override
  public void periodic() {
    // STATE TRACKING -> SmartDashboard (added)
    SmartDashboard.putString("LED/State", getLedState());
  }
}
