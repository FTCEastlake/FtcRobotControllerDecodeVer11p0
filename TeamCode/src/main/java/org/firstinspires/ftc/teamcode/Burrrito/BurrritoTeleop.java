package org.firstinspires.ftc.teamcode.Burrrito;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Burrrito.pedroPathing.BurrritoConstants;
import org.firstinspires.ftc.teamcode.Common.Vision;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "BurrritoTeleop")
public class BurrritoTeleop extends LinearOpMode {

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private Vision _vision;
    private Pose _aprilTag;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public static double MAX_POWER = 0.5;

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

            automatedDrive = false;
            if (!automatedDrive) {
                //Make the last parameter false for field-centric
                //In case the drivers want to use a "slowMode" you can scale the vectors

                //This is the normal version to use in the TeleOp
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // true = Robot Centric, false = Field Centric
                );

                //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false // true = Robot Centric, false = Field Centric
                );
            }

            if (gamepad1.a)
                AlignToAprilTag(30.0, 8.0, 0.0);

            if (gamepad1.bWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            telemetryM.addData("automatedDrive", automatedDrive);
            telemetryM.addData("isFollowerBusy", follower.isBusy());
            if (automatedDrive && (!follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

//            //Automated PathFollowing
//            if (gamepad1.aWasPressed()) {
//                follower.followPath(pathChain.get());
//                automatedDrive = true;
//            }
//
//            //Stop automated following if the follower is done
//            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//                follower.startTeleopDrive();
//                automatedDrive = false;
//            }
//
//            //Slow Mode
//            if (gamepad1.rightBumperWasPressed()) {
//                slowMode = !slowMode;
//            }
//
//            //Optional way to change slow mode strength
//            if (gamepad1.xWasPressed()) {
//                slowModeMultiplier += 0.25;
//            }
//
//            //Optional way to change slow mode strength
//            if (gamepad1.yWasPressed()) {
//                slowModeMultiplier -= 0.25;
//            }



            telemetryM.addData("position", follower.getPose());
            telemetryM.addData("velocity", follower.getVelocity());
            telemetryM.addData("automatedDrive", automatedDrive);

            _aprilTag = _vision.getFirstDetectedTag();
            if (_aprilTag != null)
            {
                telemetryM.addData("AprilTagX", _aprilTag.getX());
                telemetryM.addData("AprilTagY", _aprilTag.getY());
                telemetryM.addData("AprilTagDegrees", _aprilTag.getHeading());
            }
            //telemetryM.update();
        }


    }

    private void initRobot() throws InterruptedException {
        follower = BurrritoConstants.createFollower(hardwareMap);
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
    }

    private void AlignToAprilTag(double inchesBack, double inchesLeft, double degrees) {

        Pose currentPosition = follower.getPose();
        _aprilTag = _vision.getFirstDetectedTag();
        double forward = (currentPosition.getX() + _aprilTag.getX()) - inchesBack;
        double left = (currentPosition.getY() + _aprilTag.getY()) + inchesLeft;
        telemetryM.addData("CurX", currentPosition.getX() );
        telemetryM.addData("CurY", currentPosition.getY() );
        telemetryM.addData("AlignX", forward);
        telemetryM.addData("AlignY", left);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(forward, left))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(degrees), 0.8))
                .build();
    }

}