
package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotPrefs {

    public static boolean getDebugMode() {
        if (!Preferences.containsKey("DebugMode")) {
            Preferences.setBoolean("DebugMode", false);
        }
        return Preferences.getBoolean("DebugMode", false);
    }

    public static boolean getEncoderAndNavXDisplayed() {
        if (!Preferences.containsKey("DisplayEncoderAndNavX")) {
            Preferences.setBoolean("DisplayEncoderAndNavX", false);
        }
        return Preferences.getBoolean("DebugMode", false);
    }


    public static double getArmLengthInSpeed() {
        if (!Preferences.containsKey("ArmLengthInSpeed")) {
            Preferences.setDouble("ArmLengthInSpeed", 0.18);
        }
        return Preferences.getDouble("ArmLengthInSpeed", 0.18);
    }

    public static double getArmLengthOutSpeed() {
        if (!Preferences.containsKey("ArmLengthOutSpeed")) {
            Preferences.setDouble("ArmLengthOutSpeed", 0.18);
        }
        return Preferences.getDouble("ArmLengthOutSpeed", 0.18);
    }

    public static double getArmRotateUpSpeed() {
        if (!Preferences.containsKey("ArmRotateUpSpeed")) {
            Preferences.setDouble("ArmRotateUpSpeed", .25);
        }
        return Preferences.getDouble("ArmRotateUpSpeed", .25);
    }

    public static double getArmRotateDownSpeed() {
        if (!Preferences.containsKey("ArmRotateDownSpeed")) {
            Preferences.setDouble("ArmRotateDownSpeed", -.25);
        }
        return Preferences.getDouble("ArmRotateDownSpeed", -.25);
    }

    public static double getArmRotatePIDMovement() {
        if (!Preferences.containsKey("getArmRotatePIDMovement")) {
            Preferences.setDouble("getArmRotatePIDMovement", 0.4);
        }
        return Preferences.getDouble("getArmRotatePIDMovement", 0.4);
    }

    public static double getRotateRobotSpeed() {
        if (!Preferences.containsKey("RotateRobotSpeed")) {
            Preferences.setDouble("RotateRobotSpeed", 0.5);
        }
        return Preferences.getDouble("RotateRobotSpeed", 0.5);
    }

    public static double getRotateRobotDeadband() {
        if (!Preferences.containsKey("RotateRobotDeadband")) {
            Preferences.setDouble("RotateRobotDeadband", 5);
        }
        return Preferences.getDouble("RotateRobotDeadband", 5);
    }

    public static boolean isBlue() {
        if (!Preferences.containsKey("BlueAlliance")) {
            Preferences.setBoolean("BlueAlliance", true);
        }
        return Preferences.getBoolean("BlueAlliance", true);
    }

    public static double getBalanceP() {
        if (!Preferences.containsKey("BalanceP")) {
            Preferences.setDouble("BalanceP", 0.03);
        }
        return Preferences.getDouble("BalanceP", 0.03);
    }

    public static double getArmRotateKp() {
        if (!Preferences.containsKey("ArmRotateKp")) {
            Preferences.setDouble("ArmRotateKp", 0.03);
        }
        return Preferences.getDouble("ArmRotateKp", 0.03);

    }

    public static double getArmRotateKd() {
        if (!Preferences.containsKey("ArmRotateKd")) {
            Preferences.setDouble("ArmRotateKd", 0.05);
        }
        return Preferences.getDouble("ArmRotateKd", 0.05);

    }


    public static double getReflectiveTagOffset() {
        if (!Preferences.containsKey("ReflectiveTagOffset")) {
            Preferences.setDouble("ReflectiveTagOffset", 4.5);
        }
        return Preferences.getDouble("ReflectiveTagOffset", 4.5);
    }
}
