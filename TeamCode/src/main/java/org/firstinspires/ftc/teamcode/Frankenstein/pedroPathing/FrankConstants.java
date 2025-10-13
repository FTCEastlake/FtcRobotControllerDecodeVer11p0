package org.firstinspires.ftc.teamcode.Frankenstein.pedroPathing;

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

public class FrankConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5)     // mass is in kilograms (1kg = 2.20462 lbs)
            .forwardZeroPowerAcceleration(-35.615389529881405)
            .lateralZeroPowerAcceleration(-49.091498732147805)


            .useSecondaryHeadingPIDF(false)     // this is set to true by the call to secondaryHeadingPIDFCoefficients()
            .useSecondaryDrivePIDF(false)       // this is set to true by the call to secondaryDrivePIDFCoefficients()

            //**************************************************
            // Translational PID Coefficients
            // The translational PIDF ensures the robot follows a straight path without lateral deviation.
            //**************************************************
            // Main (coarse) correction of left/right errors.
            // Default Value: new PIDFCoefficients(0.1,0,0,0);
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.01,
                    0.0,
                    0.0,
                    0.0
            ))

            // The limit at which the translational PIDF switches between the main and
            // secondary translational PIDFs, if the secondary PID is active. Default Value: 3 (inches)
            .translationalPIDFSwitch(3)

            // Secondary translational PIDF coefficients (don't use integral).
            // Default Value: new PIDFCoefficients(0.3, 0, 0.01, 0.015)
            // Note: setting this automatically set useSecondaryTranslationalPIDF = true
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.3,
                    0.0,
                    0.01,
                    0.015
            ))


            //**************************************************
            // Heading PID Coefficients
            // The heading PIDF corrects for the robot's heading while following the path.
            //**************************************************
            // Main (coarse) heading error PIDF coefficients
            // Default Value: new PIDFCoefficients(1, 0, 0, 0.01);
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.0,
                    0.0,
                    0.0,
                    0.1
            ))

            // The limit at which the heading PIDF switches between the main and secondary heading PIDFs.
            // Default Value: Math.PI / 20 (9 degrees)
            .headingPIDFSwitch(Math.PI / 20)

            // Secondary heading error PIDF coefficients.
            // Default Value: new PIDFCoefficients(5, 0, 0.08, 0.01)
            // Note: setting this automatically set useSecondaryHeadingPIDF = true
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    5.0,
                    0.0,
                    0.08,
                    0.01
            ))


            //**************************************************
            // Drive PID Coefficients
            // The Drive PIDF manages acceleration and braking along a path, ensuring smooth motion and minimizing overshoot.
            //**************************************************
            // Main (coarse) drive PIDF coefficients
            // Default Value: new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01);
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.025,
                    0,
                    0.00001,
                    0.6,
                    0.01
            ))

            // The limit at which the heading PIDF switches between the main and secondary drive PIDFs.
            // Default Value: 20 (inches)
            .drivePIDFSwitch(20)

            // Secondary drive PIDF coefficients.
            // Default Value: new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01)
            // Note: setting this automatically set useSecondaryDrivePIDF = true
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))

            // Centripetal force to power scaling Default Value: 0.0005
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
            .useBrakeModeInTeleOp(true)     // enables active braking during TeleOp in Pedro Pathing, helping the robot resist unwanted drift when no movement is commanded.
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