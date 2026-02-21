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
    public DcMotorEx fly0, fly1;
    Gamepad gamepad;
    Limelight limelight;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // NEW: Class-level variable to store and expose RPM
    public double currentRPM = 0;

    public void init(HardwareMap hwMap, Gamepad gamepad, Limelight limelight) {
        this.gamepad = gamepad;
        this.limelight = limelight;

        try {
            fly0 = hwMap.get(DcMotorEx.class, "fly0");
            fly0.setDirection(DcMotorSimple.Direction.REVERSE);
            fly0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fly0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fly1 = hwMap.get(DcMotorEx.class, "fly1");
            fly1.setDirection(DcMotorSimple.Direction.FORWARD);
            fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) { }
    }

    // NEW: Getter so Intake can read the RPM
    public double getCurrentRPM() {
        return currentRPM;
    }

    public void controls() {
        if (fly0 == null) return;

        double targetRPM = Settings.fly_targetRPM;

        // Calculate RPM and store it in the class variable
        double currentTps = fly0.getVelocity();
        this.currentRPM = (currentTps / Settings.fly_ticksPerRev) * 60.0;

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

        fly0.setPower(Range.clip(finalPower, -1, 1));
        fly1.setPower(Range.clip(finalPower, -1, 1));

//fly0.setVelocity(targetRPM / 60.0 * 2*Math.PI, AngleUnit.RADIANS);

//fly1.setVelocity(targetRPM / 60.0 * 2*Math.PI, AngleUnit.RADIANS);
    }
}