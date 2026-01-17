package org.firstinspires.ftc.teamcode.Egg;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vision;

//@Disabled
@TeleOp(name = "EggTeleop")
public class EggTeleop extends LinearOpMode {

    private Vision _vision;
    private MecanumDrive _drive;
    //private EggIntake _intake;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        telemetry.addLine("Press A to reset Yaw");
        telemetry.update();

        while (!isStopRequested()) {

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                _drive.resetYaw();
            }

//            if (gamepad2.aWasPressed())
//                _intake.setOn();
//
//            if (gamepad2.bWasPressed())
//                _intake.setOff();

            _drive.driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

    }


    private void initRobot() throws InterruptedException {

        //_vision = new Vision(this);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        DcMotor.Direction leftFrontDirection = DcMotor.Direction.FORWARD;
        DcMotor.Direction leftRearDirection = DcMotor.Direction.FORWARD;
        DcMotor.Direction rightFrontDirection =DcMotor.Direction.REVERSE;
        DcMotor.Direction rightRearDirection = DcMotor.Direction.REVERSE;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        double maxSpeed = 1.0;  // range is 0.0 to 1.0

        _drive = new MecanumDrive(leftFrontDirection, leftRearDirection,
                rightFrontDirection, rightRearDirection,
                logoDirection, usbDirection,
                maxSpeed,this);

//        _intake = new BurrritoIntake(this);
    }


}
