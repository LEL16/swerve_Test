package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class AllianceUtil {
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red;
    } else {
      return false;
    }
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d getSpeakerPose(String teamColor) {
    return (isRedAlliance() || teamColor == "red") ? new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(180.0)) : new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0.0));
  } // 16.54
}
