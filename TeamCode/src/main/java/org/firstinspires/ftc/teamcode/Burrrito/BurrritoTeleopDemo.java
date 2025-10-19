package org.firstinspires.ftc.teamcode.Burrrito;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Vision;
import org.firstinspires.ftc.teamcode.Frankenstein.pedroPathing.FrankConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.function.Supplier;

@Disabled
@Configurable
@TeleOp(name = "BurrritoTeleopDemo")
public class BurrritoTeleopDemo extends LinearOpMode {

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private Vision _vision;
    private AprilTagDetection _aprilTag;

    ERCArmSurfNTurf _arm;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    public static double MAX_POWER = 0.5;

    // Desired offset from tag (in inches)
    private static final double DESIRED_DISTANCE = 30.0; // 24 inches from tag

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

        while (!isStopRequested()) {

            //Call this once per loop
            follower.update();
            telemetryM.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // true = Robot Centric, false = Field Centric
            );

            if (gamepad2.back) {_arm.resetArmMotorEncoders();}
            _arm.setArm();


        }


    }

    private void initRobot() throws InterruptedException {
        follower = FrankConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        _vision = new Vision(this);

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
        follower.activateAllPIDFs();
        follower.setMaxPower(MAX_POWER);     // you can configure the power for each state

        _arm = new ERCArmSurfNTurf(this);
        _arm.resetArmMotorEncoders();
    }



    /**
     * Calculate target pose to align with AprilTag
     * @param currentPose Current robot pose
     * @param detection AprilTag detection data
     * @return Target pose for alignment
     */
    private Pose calculateTargetPose(Pose currentPose, AprilTagDetection detection) {
        // Get AprilTag position relative to robot (from camera)
        double tagX = detection.ftcPose.x;
        double tagY = detection.ftcPose.y;
        double tagYaw = Math.toRadians(detection.ftcPose.yaw);

        // Convert tag position to field coordinates
        double robotHeading = currentPose.getHeading();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        // Tag position in field frame
        double tagFieldX = currentPose.getX() + (tagX * cosH - tagY * sinH);
        double tagFieldY = currentPose.getY() + (tagX * sinH + tagY * cosH);

        // Calculate angle to face the tag
        double angleToTag = Math.atan2(tagFieldY - currentPose.getY(),
                tagFieldX - currentPose.getX());

        // Calculate target position (maintain TARGET_DISTANCE from tag)
        double TARGET_DISTANCE = 30.0;
        double targetX = tagFieldX - TARGET_DISTANCE * Math.cos(angleToTag);
        double targetY = tagFieldY - TARGET_DISTANCE * Math.sin(angleToTag);

        // Target heading faces the tag
        double targetHeading = angleToTag;

        return new Pose(targetX, targetY, targetHeading);
    }

    /**
     * Calculate target pose based on AprilTag detection
     * This assumes you want to face the tag at a certain distance
     */
    private Pose calculateTargetPoseFromTag(AprilTagDetection tag, Pose currentPose) {
        // Get tag pose in robot frame
        double tagX = tag.ftcPose.x;
        double tagY = tag.ftcPose.y;
        double tagYaw = Math.toRadians(tag.ftcPose.yaw);

        // Transform to field frame
        double robotHeading = currentPose.getHeading();
        double fieldTagX = currentPose.getX() + tagX * Math.cos(robotHeading) - tagY * Math.sin(robotHeading);
        double fieldTagY = currentPose.getY() + tagX * Math.sin(robotHeading) + tagY * Math.cos(robotHeading);

        // Calculate desired position (DESIRED_DISTANCE away from tag, facing it)
        double angleToTag = Math.atan2(fieldTagY - currentPose.getY(), fieldTagX - currentPose.getX());

        double targetX = fieldTagX - DESIRED_DISTANCE * Math.cos(angleToTag);
        double targetY = fieldTagY - DESIRED_DISTANCE * Math.sin(angleToTag);
        double targetHeading = angleToTag; // Face the tag

        return new Pose(targetX, targetY, targetHeading);
    }


    /**
     * Calculate target pose based on AprilTag detection
     * This assumes you want to face the tag at a certain distance
     */
//    private Pose calculateTargetPoseFromTag(AprilTagDetection tag, Pose currentPose) {
//        // Get tag pose in robot frame
//        double tagX = tag.ftcPose.x;
//        double tagY = tag.ftcPose.y;
//        double tagYaw = Math.toRadians(tag.ftcPose.yaw);
//
//        // Transform to field frame
//        double robotHeading = currentPose.getHeading();
//        double fieldTagX = currentPose.getX() + tagX * Math.cos(robotHeading) - tagY * Math.sin(robotHeading);
//        double fieldTagY = currentPose.getY() + tagX * Math.sin(robotHeading) + tagY * Math.cos(robotHeading);
//
//        // Calculate desired position (DESIRED_DISTANCE away from tag, facing it)
//        double angleToTag = Math.atan2(fieldTagY - currentPose.getY(), fieldTagX - currentPose.getX());
//
//        double targetX = fieldTagX - DESIRED_DISTANCE * Math.cos(angleToTag);
//        double targetY = fieldTagY - DESIRED_DISTANCE * Math.sin(angleToTag);
//        double targetHeading = angleToTag; // Face the tag
//
//        return new Pose(targetX, targetY, targetHeading);
//    }

}