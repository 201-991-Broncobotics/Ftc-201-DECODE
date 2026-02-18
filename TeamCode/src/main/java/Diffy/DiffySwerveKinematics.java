package Diffy;

import static mechanisms.Settings.SweMax;

import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DiffySwerveKinematics extends Drivetrain {

    private final SwerveModule rightModule, leftModule;
    private final Telemetry telemetry;
    public double maxPower = SweMax;

    private DcMotorEx leftTop, leftBottom, rightBottom, rightTop;

    private double lastRightAngle, lastLeftAngle;

    private double[] DrivetrainVectors;
    private VoltageSensor voltageSensor;


    public DiffySwerveKinematics(
            DcMotorEx LeftTop, DcMotorEx LeftBottom,
            DcMotorEx RightBottom, DcMotorEx RightTop,
            double maxPowerLimit, Telemetry telemetry) {

        this.telemetry = telemetry;
        this.maxPower = maxPowerLimit;
        lastRightAngle = 0;
        lastLeftAngle = 0;

        DrivetrainVectors = new double[4];
        DrivetrainVectors[0] = 0; // right power
        DrivetrainVectors[1] = 0; // left power
        DrivetrainVectors[2] = 0; // right angle
        DrivetrainVectors[3] = 0; // left angle


        leftTop = LeftTop;
        leftBottom = LeftBottom;
        rightBottom = RightBottom;
        rightTop = RightTop;

        leftTop.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTop.setDirection(DcMotorSimple.Direction.FORWARD);

        leftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightModule = new SwerveModule(rightTop, rightBottom, 0, true, true);
        leftModule = new SwerveModule(leftTop, leftBottom, 0, false, false);
    }

    public void drive(double forward, double strafe, double turn, double throttle) {

        // Compute desired wheel angle and speed (field-centric math if you want to extend later)
        double driveDirection = Math.atan2(forward, strafe);// + Math.toRadians(45);
        double drivePower = Math.hypot(forward, strafe);
        forward = Math.sin(driveDirection) * drivePower;
        strafe = Math.cos(driveDirection) * drivePower;

        double A = forward + turn;
        double B = forward - turn;

        double rightPower = Math.hypot(strafe, A);
        double leftPower  = Math.hypot(strafe, B);

        double max = Math.max(1.0, Math.max(rightPower, leftPower));
        rightPower = (rightPower / max) * throttle;
        leftPower  = (leftPower / max) * throttle;

        double rightAngle = Math.toDegrees(Math.atan2(strafe, A));
        double leftAngle  = Math.toDegrees(Math.atan2(strafe, B));

        if (!(Math.abs(strafe) > 0 || Math.abs(forward) > 0 || Math.abs(turn) > 0)) {
            rightModule.setModule(lastRightAngle, 0, maxPower);
            leftModule.setModule(lastLeftAngle, 0, maxPower);
        } else {
            rightModule.setModule(rightAngle, rightPower, maxPower);
            leftModule.setModule(leftAngle, leftPower, maxPower);
            lastRightAngle = rightModule.getCurrentAngle();
            lastLeftAngle = leftModule.getCurrentAngle();
        }


        telemetry.addData("Right Angle", rightModule.getCurrentAngle());
        telemetry.addData("Left Angle", leftModule.getCurrentAngle());
    }

    public void stop() {
        rightModule.fullStopModule();
        leftModule.fullStopModule();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        rightModule.setMotorZeroBehavior(mode);
        leftModule.setMotorZeroBehavior(mode);
    }

    /** Zero both modules (used in TeleOp reset button or at INIT) */
    public void zeroModules() {
        rightModule.zeroSwerveModule();
        leftModule.zeroSwerveModule();
    }


    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {

        double forward = correctivePower.getXComponent() + pathingPower.getXComponent();
        double strafe = correctivePower.getYComponent() + pathingPower.getYComponent();
        double turn = 0;
        double throttle = 1;


        // Compute desired wheel angle and speed (field-centric math if you want to extend later)
        double driveDirection = Math.atan2(forward, strafe);// + Math.toRadians(45);
        double drivePower = Math.hypot(forward, strafe);
        forward = Math.sin(driveDirection) * drivePower;
        strafe = Math.cos(driveDirection) * drivePower;

        double A = forward + turn;
        double B = forward - turn;

        double rightPower = Math.hypot(strafe, A);
        double leftPower  = Math.hypot(strafe, B);

        double max = Math.max(1.0, Math.max(rightPower, leftPower));
        rightPower = (rightPower / max) * throttle;
        leftPower  = (leftPower / max) * throttle;

        double rightAngle = Math.toDegrees(Math.atan2(strafe, A));
        double leftAngle  = Math.toDegrees(Math.atan2(strafe, B));

        if (!(Math.abs(strafe) > 0 || Math.abs(forward) > 0 || Math.abs(turn) > 0)) {
            return new double[] {0, 0, lastRightAngle, lastLeftAngle};
        } else {
            lastRightAngle = rightModule.getCurrentAngle();
            lastLeftAngle = leftModule.getCurrentAngle();
            return new double[] {rightPower, leftPower, rightAngle, leftAngle};
        }
    }

    @Override
    public void updateConstants() {

    }

    @Override
    public void breakFollowing() {
        stop();
    }


    @Override
    public void runDrive(double[] drivePowers) {
        rightModule.setModule(drivePowers[2], drivePowers[0], maxPower);
        leftModule.setModule(drivePowers[3], drivePowers[1], maxPower);
    }


    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        super.runDrive(correctivePower, headingPower, pathingPower, robotHeading);
    }

    @Override
    public void startTeleopDrive() {
        stop();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        stop();
        setZeroPowerBehavior(
                brakeMode ?
                        DcMotor.ZeroPowerBehavior.BRAKE :
                        DcMotor.ZeroPowerBehavior.FLOAT
        );
    }


    @Override
    public double xVelocity() {
        return 0;
    }


    @Override
    public double yVelocity() {
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {

    }

    @Override
    public void setYVelocity(double yMovement) {

    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return "Right Angle: " + lastRightAngle +
                " | Left Angle: " + lastLeftAngle;
    }

}