package org.firstinspires.ftc.teamcode.Burrrito.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// For tuning: https://pedropathing.com/docs/pathing/tuning/localization
// Watch Youtube tuning video: https://www.youtube.com/watch?v=vihb2LPtSK0&t=195s

public class BurrritoConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            // Lateral correction (BurrritoTuning->Manual->Translational Tuner)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.01, 0.03))   // coefficientsTranslationalPIDF
            // Lateral correction (BurrritoTuning->Manual->Heading Tuner)
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.01, 0.03))         // coefficientsHeadingPIDF
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01 )) // coefficientsDrivePIDF
            .centripetalScaling(0.0005)     // default = 0.0005

            .forwardZeroPowerAcceleration(-33.41435813920928)   // run BurrritoTuning->Automatic->Forward Zero Power Acceleration Tuner
            .lateralZeroPowerAcceleration(-41.18010230717188)   // run BurrritoTuning->Automatic->Lateral Zero Power Acceleration Tuner
            .mass(7.0);     // mass is in kilograms (1kg = 2.20462 lbs)


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(65.94748717781127)   // run BurrritoTuning->Automatic->Forward Velocity Tuner
            .yVelocity(54.202004259965555)  // run BurrritoTuning->Automatic->Lateral Velocity Tuner
            .useBrakeModeInTeleOp(true);    // enables active braking during TeleOp in Pedro Pathing, helping the robot resist unwanted drift when no movement is commanded.


    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Note, we designate the reference point as front center of the robot, not the actual center of the robot.
            // forward pod measured in the Y direction (left is positive). This is also the xOffset value in _pinpoint.setOffsets(...)
            .forwardPodY(-2.34375)          // robot width = 12.3125", forward pod is 8.5" from left. Distance = 12.3125/2 - 8.5 = -2.34375
            // strafe pod measured in the X direction (forward is positive). This is also the yOffset value in _pinpoint.setOffsets(...)
            .strafePodX(-1.4375)           // robot length is 16", strafe pod is 9.4375" from front. Distance = 16/2 - 9.4375 = -1.4375
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // The default directions of the forward encoder = FORWARD and strafe encoder = FORWARD.
            // Because the pinpoint odometer computer is installed upside down, we need to set strafe encoder = REVERSED.
            // run BurrritoTuning->Localization->Localization Test. Forward will yield positive x value, left will yield positive y value.
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);   // REVERSED because pinpoint is installed upside down


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.5,
            1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}