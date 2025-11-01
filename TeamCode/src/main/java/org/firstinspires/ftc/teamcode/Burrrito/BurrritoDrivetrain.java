package org.firstinspires.ftc.teamcode.Burrrito;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;

@TeleOp(name = "BurrritoDrivetrain")
public class BurrritoDrivetrain extends LinearOpMode {

    private MecanumDrive _drive;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Press left bumper for max speed (1.0)");
        telemetry.addLine("Press right bumper for half speed (0.5)");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.update();

        while (!isStopRequested()) {

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                _drive.resetYaw();
            }

            if (gamepad1.left_bumper) {
                _drive.setMaxSpeed(1.0);
            }

            if (gamepad1.right_bumper) {
                _drive.setMaxSpeed(0.5);
            }

            _drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }

    }


    private void initRobot() throws InterruptedException {

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        DcMotor.Direction leftFrontDirection = DcMotor.Direction.FORWARD;
        DcMotor.Direction leftRearDirection = DcMotor.Direction.FORWARD;
        DcMotor.Direction rightFrontDirection =DcMotor.Direction.REVERSE;
        DcMotor.Direction rightRearDirection = DcMotor.Direction.REVERSE;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        double maxSpeed = 0.5;  // range is 0.0 to 1.0

        _drive = new MecanumDrive(leftFrontDirection, leftRearDirection,
                rightFrontDirection, rightRearDirection,
                logoDirection, usbDirection,
                maxSpeed,this);

    }


}
