package mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Settings {
    // --- Drive ---
    public static double SwerveKP = 0.04;
    public static double SwerveKD = 0.0005;
    public static double SweMax = .7;

    // --- Mechanism Speeds ---
    public static double FlyPower = 0.6; // Manual default power
    public static double IntakePowe = -1.0;
    public static double HighRolPow = -1.0;

    // --- Servo Positions ---
    public static double reuppos = 1;
    public static double redownpos = 0.65;

    // --- Vision / PID ---
    public static double turret_P = -0.03;
    public static double turret_I = 0;
    public static double turret_D = 0;

    // --- Auto-Aim Physics (TUNE THESE) ---
    // Angles in degrees, Heights in inches
    public static double limelightMountAngleDegrees = 20.0; // CHANGE THIS to your actual mount angle
    public static double cameraHeightInches = 10.0;        // CHANGE THIS to lens height from floor
    public static double targetHeightInches = 30.0;        // CHANGE THIS to goal height (High Basket?)

    // Flywheel Regression: Power = Base + (Distance * Multiplier)
    public static double FlywheelBasePower = 0.4;  // Minimum power needed to reach the goal at close range
    public static double FlywheelDistMult = 0.005; // How much extra power per inch of distance

    // --- RGB Degrees (Assuming 180 deg servo range for calculation) ---
    // 0 deg = 0.0, 180 deg = 1.0
    public static double SERVO_RANGE = 180.0; // Change to 270 or 300 if using a different programmer scale

    public static double RGB_RedDefault = 50;
    public static double RGB_BlueDefault = 55;
    public static double RGB_BlueWait = 60;
    public static double RGB_RedWait = 65;
    public static double RGB_RedReady = 70;
    public static double RGB_BlueReady = 75;
    public static double RGB_RedShoot = 80;
    public static double RGB_BlueShoot = 85;
    public static double RGB_Rainbow = 90;
}