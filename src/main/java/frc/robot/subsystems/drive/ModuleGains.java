package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ModuleGains {
  public static final Slot0Configs FrontLeftGains =
      new Slot0Configs()
          .withKS(0.2579)
          .withKV(0.11165)
          .withKA(0.0029128)
          .withKP(0.025353)
          .withKD(0);
  public static final Slot0Configs FrontRightGains =
      new Slot0Configs()
          .withKS(0.26304)
          .withKV(0.11309)
          .withKA(0.0035589)
          .withKP(0.0084858)
          .withKD(0);
  public static final Slot0Configs BackLeftGains =
      new Slot0Configs()
          .withKS(0.15482)
          .withKV(0.11563)
          .withKA(0.013327)
          .withKP(0.039266)
          .withKD(0);
  public static final Slot0Configs BackRightGains =
      new Slot0Configs().withKS(0.1135).withKV(0.11834).withKA(0.016269).withKP(0.059808).withKD(0);
}
