package Diffy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

import mechanisms.Settings;

public class SwerveModule {

    private final Diffy.PIDController modulePID;

    private final DoubleSupplier topMotorEncoder;

    private final DcMotorEx topMotor, bottomMotor;

    private double ModuleZeroAngle;

    private boolean usePhoton = false;
    private double LastR1Power = 0, LastR2Power = 0;

    public boolean reverseForward = false;

    public boolean reversedEncoder = false;
    public static double driveFeedBackStaticPower = 0.12; // power required from motors before robot starts moving (helps when at low speeds)


    public SwerveModule(DcMotorEx newTopMotor, DcMotorEx newBottomMotor, double startingAngle, boolean ReverseForward, boolean ReversedEncoder) { // initialize the module
        ModuleZeroAngle = startingAngle;
        topMotor = newTopMotor;
        bottomMotor = newBottomMotor;
        reversedEncoder = ReversedEncoder;
        topMotorEncoder = () -> angleDifference((((reversedEncoder)? -1 : 1) * topMotor.getCurrentPosition() / 8192.0 * 360) - ModuleZeroAngle, 0, 360);
        modulePID = new Diffy.PIDController(Settings.SwerveKP, 0, 0.0002, topMotorEncoder);
        reverseForward = ReverseForward;
    }


    public void zeroSwerveModule() {
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ModuleZeroAngle = (((reversedEncoder)? -1 : 1) * topMotor.getCurrentPosition() / 8192.0 * 360);
        modulePID.replaceDoubleSupplier(topMotorEncoder);
    }





    public double getCurrentAngle() {
        return topMotorEncoder.getAsDouble();
    }

    public double getTopMotorPower() { return topMotor.getPower(); }
    public double getBottomMotorPower() { return bottomMotor.getPower(); }


    public void setCurrentDiffyAngleTo(double newAngleZero) { ModuleZeroAngle = newAngleZero; }


    public void setModule(double angle, double speed, double maxPowerLimit) {
        modulePID.kP = Settings.SwerveKP;
        modulePID.kD = Settings.SwerveKD;

        double rotation = modulePID.getPowerWrapped(angle, 180);

        if (Math.abs(speed) > 1) speed = Math.signum(speed); // module shouldn't try to calculate speeds beyond 1 as it could mess the math up

        if (Math.abs(speed) > 0) speed = (1 - driveFeedBackStaticPower) * speed + Math.signum(speed) * driveFeedBackStaticPower;

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        speed = speed * Math.signum(Math.sin(((Math.abs(angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2));

        // maintain the correct motor speed balance
        double R1Power = ((reverseForward)? -1 : 1) * speed - rotation;
        double R2Power = ((reverseForward)? -1 : 1) * -1 * speed - rotation;
        double divider = Math.max(1, Math.max(Math.abs(R1Power) / maxPowerLimit, Math.abs(R2Power) / maxPowerLimit));

        R1Power = R1Power / divider;
        R2Power = R2Power / divider;

        topMotor.setPower(R1Power);
        LastR1Power = R1Power;

        bottomMotor.setPower(R2Power);
        LastR2Power = R2Power;
    }


    public void fullStopModule() {

        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }


    public void setMotorZeroBehavior(DcMotor.ZeroPowerBehavior mode) {
        if (!(mode == topMotor.getZeroPowerBehavior())) {
            topMotor.setZeroPowerBehavior(mode);
            bottomMotor.setZeroPowerBehavior(mode);
        }
    }



    public static double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;

    }




}