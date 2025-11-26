package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrive {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;

    // This declares the four motors needed
    private DcMotor _leftFront;
    private DcMotor _rightFront;
    private DcMotor _leftRear;
    private DcMotor _rightRear;

    private double _maxSpeed = 1.0;     // make this slower for outreaches

    // This declares the IMU needed to get the current direction the robot is facing
    private IMU _imu;

    public MecanumDrive(DcMotor.Direction leftFrontDirection,
                        DcMotor.Direction leftRearDirection,
                        DcMotor.Direction rightFrontDirection,
                        DcMotor.Direction rightRearDirection,
                        RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                        RevHubOrientationOnRobot.UsbFacingDirection usbDirection,
                        double maxSpeed,
                        LinearOpMode opMode) throws InterruptedException {

        _opMode = opMode;
        _hardwareMap = _opMode.hardwareMap;

        _leftFront = _hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = _hardwareMap.get(DcMotor.class, "rightFront");
        _leftRear = _hardwareMap.get(DcMotor.class, "leftRear");
        _rightRear = _hardwareMap.get(DcMotor.class, "rightRear");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        _rightFront.setDirection(rightFrontDirection);
        _rightRear.setDirection(rightRearDirection);
        _leftFront.setDirection(leftFrontDirection);
        _leftRear.setDirection(leftRearDirection);

        // This uses RUN_USING_ENCODER to be more accurate.
        _leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        _leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _imu = _hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        _imu.initialize(new IMU.Parameters(orientationOnRobot));

        _maxSpeed = maxSpeed;
    }

    public void resetYaw() {
        _imu.resetYaw();
    }

    public void setMaxSpeed(double maxSpeed) {
        _maxSpeed = maxSpeed;
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    private void drive(double forward, double right, double rotate) {

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        // Note: Math.max() only accepts 2 arguments.
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        _leftFront.setPower(_maxSpeed * (frontLeftPower / maxPower));
        _rightFront.setPower(_maxSpeed * (frontRightPower / maxPower));
        _leftRear.setPower(_maxSpeed * (backLeftPower / maxPower));
        _rightRear.setPower(_maxSpeed * (backRightPower / maxPower));

    }

}




