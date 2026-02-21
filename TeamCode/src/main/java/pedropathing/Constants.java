package pedropathing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Diffy.DiffySwerveKinematics;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.02229);
         //   .xVelocity(-34.29648);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6) // Need to remeasure lol
            .strafePodX(6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pin0")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        DiffySwerveKinematics diffy = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                1.0,
                telemetry );
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .setDrivetrain(diffy)

                .build();
    }

}