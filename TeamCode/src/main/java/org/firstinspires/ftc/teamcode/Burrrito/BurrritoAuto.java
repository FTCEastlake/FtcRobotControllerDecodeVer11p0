package org.firstinspires.ftc.teamcode.Burrrito;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Burrrito.pedroPathing.BurrritoConstants;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "BurrritoAuto")
//@Autonomous(name = "BurrritoAuto")
public class BurrritoAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;

    // Poses are points on the field coordinate system that define the (x, y, heading) of your robot during the autonomous.
    // Pedro’s coordinate system spans an interval of [0, 144] on both the x and y axes,
    // with (0, 0) defined as the bottom-right corner of the field.
    // Note1: these x and y values are in inches. Max of 144 inches because the field is 12 ft by 12 ft.
    // Note2: positive x is forward, negative x is backwards
    // Note3: positive y is left, negative y is right
    // Note4: positive rotation is counter clockwise (range is 0 to 180), negative rotation is clockwise (range is 0 to -180)

    private Pose startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // start Pose of our robot.
    private Pose scorePose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // scoring position
    private Pose pickup1Pose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // first pickup
    private Pose pickup2Pose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // second pickup
    private Pose pickup3Pose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // third pickup

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public static double MAX_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        selectConfiguration();

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);

        while (!isStopRequested()) {

            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }

    }

    private List<String> autoConfig = new ArrayList<>();
    private int selectedIndex = 0;
    private void selectConfiguration() {

        autoConfig.add("Red__A");
        autoConfig.add("Red__B");
        autoConfig.add("Blue__A");
        autoConfig.add("Blue__B");

        // Menu loop
        // This will continuously loop through the list.
        // Just leave it at the selection you want before the start condition.
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Use D-pad to select test");
            //telemetry.addLine("Press A to confirm");
            telemetry.addLine("Selected: " + autoConfig.get(selectedIndex));
            telemetry.update();

            if (gamepad1.dpad_up) {
                selectedIndex = (selectedIndex - 1 + autoConfig.size()) % autoConfig.size();
                sleep(200);
            } else if (gamepad1.dpad_down) {
                selectedIndex = (selectedIndex + 1) % autoConfig.size();
                sleep(200);
            }
//            else if (gamepad1.a) {
//                break;
//            }
        }

        // Note2: positive x is forward, negative x is backwards
        // Note3: positive y is left, negative y is right
        // Note4: positive rotation is counter clockwise (range is 0 to 180), negative rotation is clockwise (range is 0 to -180)
        double tileScaleFactor = 0.4;   // this should be 1.0. Setting this to 0.4 for debug in small space.
        switch (selectedIndex) {
            case 0: // Red_PositionA
                // Starts at 2 tiles from the top left referenced to red team
                startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // Start Pose of our robot.
                scorePose = new Pose(0.0 * tileScaleFactor, -48.0 * tileScaleFactor, Math.toRadians(45)); // move to -48 inches right, turn 45 degrees counter clockwise
                pickup1Pose = new Pose(12.0 * tileScaleFactor, -60.0 * tileScaleFactor, Math.toRadians(0)); // move to -60 inches right, turn 0 degrees
                pickup2Pose = new Pose(12.0 * tileScaleFactor, -84.0 * tileScaleFactor, Math.toRadians(0)); // move to -84 inches right, turn 0 degrees
                pickup3Pose = new Pose(12.0 * tileScaleFactor, -108.0 * tileScaleFactor, Math.toRadians(0)); // move to -108 inches right, turn 0 degrees
            break;
            case 1: // Red_PositionB
                // Starts at 3 tiles from the top left referenced to red team
                startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // Start Pose of our robot.
                scorePose = new Pose(24.0 * tileScaleFactor, -48.0 * tileScaleFactor, Math.toRadians(45)); // move to -48 inches right, turn 45 degrees counter clockwise
                pickup1Pose = new Pose(36.0 * tileScaleFactor, -60.0 * tileScaleFactor, Math.toRadians(0)); // move to -60 inches right, turn 0 degrees
                pickup2Pose = new Pose(36.0 * tileScaleFactor, -84.0 * tileScaleFactor, Math.toRadians(0)); // move to -84 inches right, turn 0 degrees
                pickup3Pose = new Pose(36.0 * tileScaleFactor, -108.0 * tileScaleFactor, Math.toRadians(0)); // move to -108 inches right, turn 0 degrees
                break;
            case 2: // Blue_PositionA
                // Starts at 2 tiles from the top right referenced to blue team
                startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // Start Pose of our robot.
                scorePose = new Pose(0.0 * tileScaleFactor, 48.0 * tileScaleFactor, Math.toRadians(-45)); // move to 48 inches right, turn 45 degrees clockwise
                pickup1Pose = new Pose(12.0 * tileScaleFactor, 60.0 * tileScaleFactor, Math.toRadians(0)); // move to -60 inches right, turn 0 degrees
                pickup2Pose = new Pose(12.0 * tileScaleFactor, 84.0 * tileScaleFactor, Math.toRadians(0)); // move to -84 inches right, turn 0 degrees
                pickup3Pose = new Pose(12.0 * tileScaleFactor, 108.0 * tileScaleFactor, Math.toRadians(0)); // move to -108 inches right, turn 0 degrees
                break;
            case 3: // Blue_PositionB
                // Starts at 3 tiles from the top right referenced to blue team
                startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // Start Pose of our robot.
                scorePose = new Pose(24.0 * tileScaleFactor, 48.0 * tileScaleFactor, Math.toRadians(-45)); // move to 48 inches right, turn 45 degrees clockwise
                pickup1Pose = new Pose(36.0 * tileScaleFactor, 60.0 * tileScaleFactor, Math.toRadians(0)); // move to 60 inches right, turn 0 degrees
                pickup2Pose = new Pose(36.0 * tileScaleFactor, 84.0 * tileScaleFactor, Math.toRadians(0)); // move to 84 inches right, turn 0 degrees
                pickup3Pose = new Pose(36.0 * tileScaleFactor, 108.0 * tileScaleFactor, Math.toRadians(0)); // move to 108 inches right, turn 0 degrees
                break;
            default: break;
        }

        buildPaths();
        follower.setStartingPose(startPose);
    }

    private void initRobot() throws InterruptedException {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = BurrritoConstants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(MAX_POWER);     // you can configure the power for each state
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO: Score Preload */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: pick up balls */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO: Score balls */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: pick up balls */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO: Score balls */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: pick up balls */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO: Score balls */
                    sleep(3000); // pauses for 3000 milliseconds (3 second)

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
