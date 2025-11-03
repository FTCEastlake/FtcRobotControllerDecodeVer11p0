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

public class BurrritoConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            // We are not using dual PID system until we're more experienced in single PID system.
            //.useSecondaryTranslationalPIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryDrivePIDF(true)

            // Lateral correction (BurrritoTuning->Manual->Translational Tuner)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.0, 0))

            .forwardZeroPowerAcceleration(-35.6123)     // run BurrritoTuning->Automatic->Forward Zero Power Acceleration Tuner
            .lateralZeroPowerAcceleration(-35.4604)     // run BurrritoTuning->Automatic->Lateral Zero Power Acceleration Tuner
            .mass(5.0);     // mass is in kilograms (1kg = 2.20462 lbs)


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
            .xVelocity(67.8704)     // run BurrritoTuning->Automatic->Forward Velocity Tuner
            .yVelocity(54.4133)     // run BurrritoTuning->Automatic->Lateral Velocity Tuner
            .useBrakeModeInTeleOp(true);     // enables active braking during TeleOp in Pedro Pathing, helping the robot resist unwanted drift when no movement is commanded.


    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Note, we designate the reference point as front center of the robot, not the actual center of the robot.
            // forward pod measured in the Y direction (left is positive). This is also the xOffset value in _pinpoint.setOffsets(...)
            .forwardPodY(-2.3125)
            // strafe pod measured in the X direction (forward is positive). This is also the yOffset value in _pinpoint.setOffsets(...)
            .strafePodX(-9.5)           // strafe pod measured in the X direction (forward is positive)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // The default directions of the forward encoder = FORWARD and strafe encoder = FORWARD.
            // Because the pinpoint odometer computer is installed upside down, we need to set strafe encoder = REVERSED.
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);   // REVERSED because pinpoint is installed upside down


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.995,
//            0.1,
//            0.1,
//            0.009,
//            50,
//            1.25,
//            10,
//            1
//    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}