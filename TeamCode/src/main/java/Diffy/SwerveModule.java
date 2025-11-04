package Diffy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Diffy Swerve Module with encoder feedback.
 * Uses top and bottom motor encoder positions to estimate steering angle.
 */
public class SwerveModule {

    private final DcMotorEx topMotor;
    private final DcMotorEx bottomMotor;
    private double zeroAngleDeg = 0.0;   // zero offset for steering angle
    private double bottomMotorCoeff = 1.0;

    // Example constants (adjust to your hardware)
    private static final double TICKS_PER_REV = 537.7;  // for goBILDA 5202 motor
    private static final double GEAR_RATIO = 1.0;       // 1:1 between motor and steering rotation

    public SwerveModule(DcMotorEx topMotor, DcMotorEx bottomMotor, double bottomMotorCoeff) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.bottomMotorCoeff = bottomMotorCoeff;

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Command the module to a target direction & speed.
     */
    public void setModule(double targetAngleDeg, double speed, double maxPowerLimit) {
        double currentAngle = getCurrentAngle();
        double deltaAngle = targetAngleDeg - currentAngle;

        // Normalize to [-180, 180]
        while (deltaAngle > 180) deltaAngle -= 360;
        while (deltaAngle < -180) deltaAngle += 360;

        double rotation = deltaAngle / 180.0; // proportion of rotation to reach target

        double r1 = speed + rotation;
        double r2 = -speed + rotation;

        double max = Math.max(1.0, Math.max(Math.abs(r1), Math.abs(r2)));
        r1 = (r1 / max) * maxPowerLimit;
        r2 = (r2 / max) * maxPowerLimit;

        topMotor.setPower(r1);
        bottomMotor.setPower(bottomMotorCoeff * r2);
    }

    /** Reads current steering angle based on encoder difference. */
    public double getCurrentAngle() {
        // Diffy principle: steering angle = difference between top and bottom encoders
        double diffTicks = topMotor.getCurrentPosition() - bottomMotor.getCurrentPosition();
        double revolutions = (diffTicks / TICKS_PER_REV) / GEAR_RATIO;
        double angleDeg = revolutions * 360.0;

        // Apply zero offset
        return angleDeg - zeroAngleDeg;
    }

    /** Set current angle as "forward = 0°" */
    public void zeroSwerveModule() {
        zeroAngleDeg = (topMotor.getCurrentPosition() - bottomMotor.getCurrentPosition())
                / TICKS_PER_REV * 360.0;
    }

    /** Stops both motors */
    public void fullStopModule() {
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    /** Sets brake/float mode */
    public void setMotorZeroBehavior(DcMotor.ZeroPowerBehavior mode) {
        topMotor.setZeroPowerBehavior(mode);
        bottomMotor.setZeroPowerBehavior(mode);
    }
}