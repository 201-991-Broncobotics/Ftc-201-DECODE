package mechanisms;

import com.acmerobotics.dashboard.config.Config;
@Config
public class Settings {

    // UPDATED: Default is now 2
    public static int numControllers = 1;

    // --- VISION ---
    public static double CAMERA_HEIGHT_IN = 16.5;
    public static double TARGET_HEIGHT_IN = 31.0;
    public static double CAMERA_ANGLE_DEG = 10.0;
    public static double aimOffsetScale = -1;
    public static double CAMERA_OFFSET_FROM_CENTER = -5.0;

    // --- FLYWHEEL FORMULA ---
    // Equation: RPM = 17.37098 * dist + 1242.65941
    public static double fly_A = 0.028;
    public static double fly_B = 7.5;
    public static double fly_C = 1900;

    // --- FLYWHEEL PID ---
    public static double fly_targetRPM = 100;
    public static double fly_ticksPerRev = 28.0;
    public static double fly_kP = 1.4;
    public static double fly_kI = 0;
    public static double fly_kD = 0.001;
    public static double fly_pidDivisor = 1000.0;

    public static double fly_farPreset = 2720;
    public static double fly_closePreset = 2200;
    public static double FlyPower = 1;

    // --- TURRET ---
    public static double tur_maxSpeed = 1.0;
    public static double tur_tolerance = 1.0;
    public static double tur_kP = -0.00185;
    public static double tur_kI = 0;
    public static double tur_kD = -0.00085;
    public static double tur_static_F = 0.07;
    public static double tur_searchSpeed = 0; //should be -.15 to activate

    // --- SWERVE ---
    public static double SwerveKP = 0.04;
    public static double SwerveKD = 0.0005;
    public static double SweMax = 1;

    // --- INTAKE & SERVO POSITIONS ---
    public static double IntakePowe = -1;
    public static double HighRolPow = 1;

    public static double reuppos = 0.63;
    public static double redownpos = 1;

    public static double flick_UP = 0.43;
    public static double flick_DOWN = 0.95;

    public static int shoot_delay_ms = 250;

    // --- RGB ---
    public static double rgb_default = 1.0;
}