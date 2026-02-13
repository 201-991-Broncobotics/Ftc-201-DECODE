package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import sensors.Limelight;

public class Flywheel {
    public DcMotorEx fly;
    Gamepad gamepad;
    Limelight limelight;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap, Gamepad gamepad, Limelight limelight) {
        this.gamepad = gamepad;
        this.limelight = limelight;

        try {
            fly = hwMap.get(DcMotorEx.class, "flyM");
            fly.setDirection(DcMotorSimple.Direction.REVERSE);
            fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) { }
    }

    public void controls() {
        if (fly == null) return;

        // Updated to flattened Settings
        double targetRPM = Settings.fly_targetRPM;

        // Calculate RPM
        double currentTps = fly.getVelocity();
        double currentRPM = (currentTps / Settings.fly_ticksPerRev) * 60.0;

        // PID
        double error = targetRPM - currentRPM;

        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.2) dt = 0.02;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Feedforward + PID
        double kF = targetRPM / 6000.0;
        double pidOutput = (error * Settings.fly_kP) +
                (integralSum * Settings.fly_kI) +
                (derivative * Settings.fly_kD);

        double scaledPid = pidOutput / Settings.fly_pidDivisor;
        double finalPower = kF + scaledPid;

        if (Math.abs(targetRPM) < 50) {
            finalPower = 0;
            integralSum = 0;
        }

        fly.setPower(Range.clip(finalPower, -1, 1));
    }
}