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

@Configurable
@TeleOp(name = "BurrritoTeleopArmDemo")
public class BurrritoTeleopArmDemo extends LinearOpMode {


    ERCArmSurfNTurf _arm;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    public static double MAX_POWER = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad2.back) {_arm.resetArmMotorEncoders();}
            _arm.setArm();
        }


    }

    private void initRobot() throws InterruptedException {

        // Notes: Expansion hub was removed so had to move the arm motors and servo to the control hub.
        // Expansion hub
        // Motors:
        //    port 0: gobilda 5202/3/4 rotationMotor
        //    port 1: gobilda 5202/3/4 slideMotor
        // Servo
        //    port 0: Servo clawServo
        
        _arm = new ERCArmSurfNTurf(this);
        _arm.resetArmMotorEncoders();
    }




}