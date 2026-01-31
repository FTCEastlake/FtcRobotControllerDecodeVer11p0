package org.firstinspires.ftc.teamcode.Egg;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Burrrito.BurrritoAuto_Blue;
import org.firstinspires.ftc.teamcode.Egg.pedroPathing.EggConstants;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@Configurable
@TeleOp(name = "EggAuto")
//@Autonomous(name = "EggAuto")
public class EggAuto extends LinearOpMode {

    // panels: http://192.168.43.1:8001/
    private Follower _follower;
    private TelemetryManager _telemetryM;
    //private EggIntake _intake;

    // Poses are points on the field coordinate system that define the (x, y, heading) of your robot during the autonomous.
    // Pedro’s coordinate system spans an interval of [0, 144] on both the x and y axes,
    // with (0, 0) defined as the bottom-right corner of the field.
    // Note1: these x and y values are in inches. Max of 144 inches because the field is 12 ft by 12 ft.
    // Note2: positive x is forward, negative x is backwards
    // Note3: positive y is left, negative y is right
    // Note4: positive rotation is counter clockwise (range is 0 to 180), negative rotation is clockwise (range is 0 to -180)

    private Pose _startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));  // start Pose of our robot.
    private Pose _scorePose = new Pose(-0.0, -20.0, Math.toRadians(-45.0));  // scoring position

    private Pose _startPickup1Pose = new Pose(0.0, -40.0, Math.toRadians(0));  // first pickup
    private Pose _stopPickup1Pose = new Pose(-10.0, -40.0, Math.toRadians(0));  // first pickup

    private Pose _startPickup2Pose = new Pose(-10.0, -30.0, Math.toRadians(0));  // second pickup
    private Pose _stopPickup2Pose = new Pose(-15.0, -30.0, Math.toRadians(0));  // second pickup

    private Pose _startPickup3Pose = new Pose(-10.0, -10.0, Math.toRadians(0));  // third pickup
    private Pose _stopPickup3Pose = new Pose(-15.0, -10.0, Math.toRadians(0));  // third pickup


    private enum PathStates {
        start,
        scoring,
        startPickup1,
        startPickup2,
        startPickup3,
        stopPickup1,
        stopPickup2,
        stopPickup3,
        stop
    }
    private PathStates _pathState;

    public static double MAX_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();


        while (!isStopRequested()) {

            //*********************************************************
            // Move to scoring position and shoot preloaded balls
            //*********************************************************
            move(_startPose, _scorePose, MAX_POWER);
            shootBalls();


            //*********************************************************
            // Move to 1st pickup position and intake balls
            //*********************************************************
            move(_scorePose, _startPickup1Pose, MAX_POWER);
            //_intake.setOn();
            move(_startPickup1Pose, _stopPickup1Pose, 0.2);
            //_intake.setOff();

            // Move to scoring position and shoot balls
            move(_stopPickup1Pose, _scorePose, MAX_POWER);
            shootBalls();


//            //*********************************************************
//            // Move to 2nd pickup position and intake balls
//            //*********************************************************
//            move(_scorePose, _startPickup2Pose, MAX_POWER);
//            //_intake.setOn();
//            move(_startPickup2Pose, _stopPickup2Pose, 0.2);
//            //_intake.setOff();
//
//            // Move to scoring position and shoot balls
//            move(_stopPickup2Pose, _scorePose, MAX_POWER);
//            shootBalls();
//
//
//            //*********************************************************
//            // Move to 3rd pickup position and intake balls
//            //*********************************************************
//            move(_scorePose, _startPickup3Pose, MAX_POWER);
//            //_intake.setOn();
//            move(_startPickup3Pose, _stopPickup3Pose, 0.2);
//            //_intake.setOff();
//
//            // Move to scoring position and shoot balls
//            move(_stopPickup3Pose, _scorePose, MAX_POWER);
//            shootBalls();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", _pathState);
            telemetry.addData("x", _follower.getPose().getX());
            telemetry.addData("y", _follower.getPose().getY());
            telemetry.addData("heading", _follower.getPose().getHeading());
            telemetry.update();

            break;
        }

    }

    private void move(Pose startPosition, Pose endPosition, double power) {

        Path pathToMove = new Path(new BezierLine(startPosition, endPosition));
        pathToMove.setLinearHeadingInterpolation(startPosition.getHeading(), endPosition.getHeading());

        _follower.setMaxPower(power);
        _follower.followPath(pathToMove,true);
        while (_follower.isBusy()) { _follower.update(); }
    }

    private void shootBalls() {
        /* TODO: Shoot preloaded balls */
        //_intake.setOn();
        sleep(3000); // simulate shooting the ball
        //_intake.setOff();
    }


    private void initRobot() throws InterruptedException {

        _telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //_intake = new EggIntake(this);

        _follower = EggConstants.createFollower(hardwareMap);
        _follower.setStartingPose(_startPose);
    }

}
