package mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Settings {

    // --- 1. ROBOT SETUP ---
    public static int numControllers = 1;

    // --- 2. VISION CONSTANTS ---
    // Measure these precisely!
    public static double CAMERA_HEIGHT_IN = 16.5;
    public static double TARGET_HEIGHT_IN = 31.0;
    public static double CAMERA_ANGLE_DEG = 10.0;
    public static double CAMERA_OFFSET_FROM_CENTER = -5.0;

    // Aim Offset: -1.1 means "Aim 11 deg left if tag is 10 deg right"
    public static double aimOffsetScale = -1.1;

    // --- 3. FLYWHEEL TUNING ---
    // Quadratic: RPM = (A * dist^2) + (B * dist) + C
    public static double fly_A = 0.0;
    public static double fly_B = 0.0;
    public static double fly_C = 3000.0;

    public static double fly_targetRPM = 4500;
    public static double fly_ticksPerRev = 28.0;

    // Flywheel PID
    public static double fly_kP = 0.001;
    public static double fly_kI = 0.0;
    public static double fly_kD = 0.0;
    public static double fly_pidDivisor = 1000.0;

    // Flywheel Presets
    public static double fly_farPreset = 4500;
    public static double fly_closePreset = 1800;

    // --- 4. TURRET TUNING ---
    public static double tur_maxSpeed = 1.0;
    public static double tur_tolerance = 1.0;

    public static double tur_kP = -0.00185;
    public static double tur_kI = -0.01;
    public static double tur_kD = -0.00085;
    public static double tur_static_F = 0.07;

    // --- 5. SWERVE & MISC ---
    public static double SwerveKP = 0.04;
    public static double SwerveKD = 0.0005;
    public static double SweMax = 1;

    // Legacy/Other
    public static double FlyPower = 6000;
    public static double IntakePowe = -1;
    public static double HighRolPow = -1;
    public static double reuppos = 0.63;
    public static double redownpos = 1;
    public static double RGBCOLOR = 0;
    public static double padle = 0;
    public static int pipeline = 0;
}