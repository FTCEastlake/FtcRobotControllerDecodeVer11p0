package org.firstinspires.ftc.teamcode.Burrrito.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@Disabled
@Configurable
@TeleOp(name = "BurrritoPedroDrivetrain")
public class BurrritoPedroDrivetrain extends LinearOpMode {

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    public static double MAX_POWER = 0.5;

    // Desired offset from tag (in inches)
    private static final double DESIRED_DISTANCE = 30.0; // 30 inches from tag

    @Override
    public void runOpMode() throws InterruptedException {

        // Note: MecanumConstants parameters are in the pedroPathing/constants file.
        initRobot();

        waitForStart();

        follower.startTeleopDrive();

        while (!isStopRequested()) {

            //Call this once per loop
            follower.update();
            telemetryM.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * MAX_POWER,
                    -gamepad1.left_stick_x * MAX_POWER,
                    -gamepad1.right_stick_x * MAX_POWER,
                    false // true = Robot Centric, false = Field Centric
            );

        }

    }


    private void initRobot() throws InterruptedException {
        follower = BurrritoConstants.createFollower(hardwareMap);
        follower.setStartingPose(follower.getPose());
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.activateAllPIDFs();
        follower.setMaxPower(MAX_POWER);     // you can configure the power for each state
    }


}