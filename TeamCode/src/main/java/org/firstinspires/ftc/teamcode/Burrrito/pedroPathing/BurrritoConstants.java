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
            .mass(11.5)     // mass is in kilograms (1kg = 2.20462 lbs)
            .forwardZeroPowerAcceleration(-35.615389529881405)
            .lateralZeroPowerAcceleration(-49.091498732147805)

            //**************************************************
            // Translational PID Coefficients
            //**************************************************
            // The limit (in inches) at which the translational PIDF switches between the main and secondary
            // translational PIDFs, if the secondary PID is active. Default Value: 3
            // Ex) translationalPIDFSwitch(3) means
            //        outside of 3 inches, the first PID is used to correct the robot,
            //        inside of 3 inches, the second PID is used to correct the robot
            .translationalPIDFSwitch(3)
            // This is the coarse (large) PID tuning for translational (left/right) correction.
            // This is just meant to correct the robot to and bring it within the secondary PID's range.
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03,
                    0,
                    0,
                    0.015
            ))
            // This is the fine (small) PID tuning for translational (left/right) correction.
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.005,
                    0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    0.0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    0.0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.3378310316191)
            .yVelocity(52.962789700725885);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.75)      // forward pod measured in the Y direction (left is positive)
            .strafePodX(-2.0)       // strafe pod measured in the X direction (forward is positive)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}