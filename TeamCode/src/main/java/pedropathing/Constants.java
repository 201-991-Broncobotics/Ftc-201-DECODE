package pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // Mass in KG (Make sure this is accurate for physics predictions)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.0327272);

    // Speed and Acceleration constraints
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,  // Zero Power Acceleration Multiplier
            100.0, // Max Speed (inches/sec) - Diffy swerve is fast, but start conservative
            10.0,  // Max Acceleration (inches/sec^2) - Keep this low for testing!
            1.0    // Centripetal Scaling
    );

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.0)   // 5 inches to the RIGHT of center
            .strafePodX(0.5)     // 0.5 inches FORWARD of center
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            // CHECK THIS: Did you buy the "Swingarm" (yellow) or "4-Bar" (older) pods?
            // Most new kits are Swingarm. 4-Bar has different ticks/mm.
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}

