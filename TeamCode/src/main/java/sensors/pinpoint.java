package sensors;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

// PEDRO PATHING IMPORTS
import com.pedropathing.localization.Localizer;
import com.pedropathing.ftc.localization.constants.PinpointConstants;

// FTC IMPORTS
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class pinpoint implements Localizer {
    private GoBildaPinpointDriver driver;
    private Pose startPose;

    public pinpoint(HardwareMap hardwareMap, PinpointConstants constants) {
        // Initialize hardware
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pin0");

        // Set Offsets (X, Y in inches)
        driver.setOffsets(3, 10, DistanceUnit.INCH);

        // Set Encoder Resolution
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Set Directions
        driver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        driver.resetPosAndIMU();
        startPose = new Pose(0,0,0);

    }

    @Override
    public Pose getPose() {
        // Convert GoBilda Pose2D to Pedro Pose
        Pose2D rawPose = driver.getPosition();
        return new Pose(
                rawPose.getX(DistanceUnit.INCH),
                rawPose.getY(DistanceUnit.INCH),
                rawPose.getHeading(AngleUnit.RADIANS)
        );
    }
    private Pose lastPose = new Pose(0,0,0);
    private long lastTime = System.nanoTime();
    private Pose currentVelocity = new Pose(0,0,0);



    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(
                currentVelocity.getX(),
                currentVelocity.getY()
        );
    }



    @Override
    public void setStartPose(Pose pose) {
        this.startPose = pose;
        // Pinpoint requires a full reset to set a new zero, or we just set the current position
        driver.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }

    @Override
    public void setPose(Pose pose) {
        driver.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }

    @Override
    public void update() {
        driver.update();

        Pose currentPose = getPose();

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // seconds

        if (dt > 0) {
            double vx = (currentPose.getX() - lastPose.getX()) / dt;
            double vy = (currentPose.getY() - lastPose.getY()) / dt;
            double vHeading = (currentPose.getHeading() - lastPose.getHeading()) / dt;

            currentVelocity = new Pose(vx, vy, vHeading);
        }

        lastPose = currentPose;
        lastTime = now;
    }

    @Override
    public double getTotalHeading() {
        return driver.getPosition().getHeading(AngleUnit.RADIANS);
    }

    // These methods are required by the Abstract Localizer class
    // Since Pinpoint handles its own forward/lateral logic, we return 1.0 (no scaling needed)
    @Override
    public double getForwardMultiplier() { return 1.0; }

    @Override
    public double getLateralMultiplier() { return 1.0; }

    @Override
    public double getTurningMultiplier() { return 1.0; }

    // Optional: Expose IMU reset if needed
    public void resetIMU() { driver.recalibrateIMU(); }

    @Override
    public double getIMUHeading() {
        return driver.getPosition().getHeading(AngleUnit.RADIANS);
    }


    @Override
    public boolean isNAN() {
        return false;
    }

    @Override
    public void setX(double x) {
        Localizer.super.setX(x);
    }

    @Override
    public void setY(double y) {
        Localizer.super.setY(y);
    }

    @Override
    public void setHeading(double heading) {
        Localizer.super.setHeading(heading);
    }
}