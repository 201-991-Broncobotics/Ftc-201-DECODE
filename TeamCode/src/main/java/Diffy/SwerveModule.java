package Diffy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class SwerveModule {

    private final Diffy.PIDController modulePID;

    private final DoubleSupplier topMotorEncoder;

    private final DcMotorEx topMotor, bottomMotor;

    private double ModuleZeroAngle;

    private boolean usePhoton = false;
    private double LastR1Power = 0, LastR2Power = 0;

    public double bottomMotorCoeff = 1;

    public static double driveFeedBackStaticPower = 0.12; // power required from motors before robot starts moving (helps when at low speeds)


    public SwerveModule(DcMotorEx newTopMotor, DcMotorEx newBottomMotor, double startingAngle, double BottomMotorCoeff) { // initialize the module
        ModuleZeroAngle = startingAngle;
        topMotor = newTopMotor;
        bottomMotor = newBottomMotor;
        topMotorEncoder = () -> angleDifference((topMotor.getCurrentPosition() / 8192.0 * 360) - ModuleZeroAngle, 0, 360);
        modulePID = new Diffy.PIDController(0.007, 0, 0.0002, topMotorEncoder);
        bottomMotorCoeff = BottomMotorCoeff;
    }


    public void zeroSwerveModule() {
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ModuleZeroAngle = (topMotor.getCurrentPosition() / 8192.0 * 360);
        modulePID.replaceDoubleSupplier(topMotorEncoder);
    }





    public double getCurrentAngle() {
        return topMotorEncoder.getAsDouble();
    }

    public double getTopMotorPower() { return topMotor.getPower(); }
    public double getBottomMotorPower() { return bottomMotor.getPower(); }


    public void setCurrentDiffyAngleTo(double newAngleZero) { ModuleZeroAngle = newAngleZero; }


    public void setModule(double angle, double speed, double maxPowerLimit) {

        double rotation = modulePID.getPowerWrapped(angle, 180);

        if (Math.abs(speed) > 1) speed = Math.signum(speed); // module shouldn't try to calculate speeds beyond 1 as it could mess the math up

        if (Math.abs(speed) > 0) speed = (1 - driveFeedBackStaticPower) * speed + Math.signum(speed) * driveFeedBackStaticPower;

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        speed = speed * Math.signum(Math.sin(((Math.abs(angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2));

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -1*speed + rotation;
        double divider = Math.max(1, Math.max(R1Power / maxPowerLimit, R2Power / maxPowerLimit));

        R1Power =  R1Power / divider;
        R2Power =  R2Power / divider;

        topMotor.setPower(R1Power);
        LastR1Power = R1Power;

        bottomMotor.setPower(bottomMotorCoeff * R2Power);
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

