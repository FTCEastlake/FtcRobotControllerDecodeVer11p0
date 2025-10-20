package org.firstinspires.ftc.teamcode.Frankenstein;

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
@TeleOp(name = "FrankTeleop")
public class FrankTeleop extends LinearOpMode {

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private Vision _vision;
    private AprilTagDetection _aprilTag;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    public static double MAX_POWER = 0.5;

    // Desired offset from tag (in inches)
    private static final double DESIRED_DISTANCE = 30.0; // 30 inches from tag

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        follower.startTeleopDrive();

        while (!isStopRequested()) {

            //Call this once per loop
            follower.update();
            telemetryM.update();

            if (!automatedDrive) {
                //Make the last parameter false for field-centric
                //In case the drivers want to use a "slowMode" you can scale the vectors

                //This is the normal version to use in the TeleOp
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * MAX_POWER,
                        -gamepad1.left_stick_x * MAX_POWER,
                        -gamepad1.right_stick_x * MAX_POWER,
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


            _aprilTag = _vision.getFirstDetectedTag();
            if (_aprilTag != null) {
                telemetryM.addData("AprilTagX left(-), right(+) (inches)", _aprilTag.ftcPose.x);
                telemetryM.addData("AprilTagY front(+), back(-) (inches)", _aprilTag.ftcPose.y);
                telemetryM.addData("AprilTagRange (inches)", _aprilTag.ftcPose.range);
                telemetryM.addData("AprilTagYaw counter clockwise(+) (degrees)", _aprilTag.ftcPose.yaw);
                telemetryM.addData("AprilTagBearing counter clockwise(+) (degrees)", _aprilTag.ftcPose.bearing);
            }




            // Automated PathFollowing
            if (gamepad1.aWasPressed()) {
                Pose currentPose = follower.getPose();
                Pose targetPose = _vision.AlignToAprilTag(currentPose, DESIRED_DISTANCE);

                pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                        .addPath(new Path(new BezierLine(currentPose, targetPose)))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                        .build();
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            // Stop automated following if the follower is done.
            // The automated PathFollowing can be cancelled by pressing B button.
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }


            // telemetryM.addData("position", follower.getPose());
            Pose currentPose = follower.getPose();
            telemetryM.addData("curHeading: ", currentPose.getHeading());
            String formattedPose = String.format("x=%.5g, y=%.5g, h=%.5g", currentPose.getX(), currentPose.getY(), currentPose.getHeading());
            telemetryM.addData("position: ", formattedPose);
            //telemetryM.addData("position: ", follower.getPose());

            // telemetryM.addData("velocity", follower.getVelocity());
            Vector currentVelocity = follower.getVelocity();
            String formattedVelocity = String.format("magnitude=%.5f, theta=%.5f, xComponent=%.5f, yComponent=%.5f",
                    currentVelocity.getMagnitude(), currentVelocity.getTheta(), currentVelocity.getXComponent(), currentVelocity.getYComponent());
            telemetryM.addData("velocity", formattedVelocity);
            telemetryM.addData("automatedDrive", automatedDrive);

        }


    }

    private void initRobot() throws InterruptedException {
        follower = FrankConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        _vision = new Vision(this);

        follower.activateAllPIDFs();
        follower.setMaxPower(MAX_POWER);     // you can configure the power for each state
    }



}