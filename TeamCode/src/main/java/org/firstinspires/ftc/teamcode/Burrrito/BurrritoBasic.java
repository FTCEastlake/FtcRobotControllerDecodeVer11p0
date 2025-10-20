package org.firstinspires.ftc.teamcode.Burrrito;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Burrrito.pedroPathing.BurrritoConstants;

import java.util.function.Supplier;

//@Disabled
@Configurable
@TeleOp(name = "BurrritoBasic")
public class BurrritoBasic extends LinearOpMode {

    //**************************************************************
    // Control Hub:
    //    USB port: "webcam 1"
    //    Motor port0: "leftFront"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "rightFront" (GoBILDA 5202/3/4 series)
    //    Motor port2: "leftRear"   (GoBILDA 5202/3/4 series)
    //    Motor port3: "backRight"  (GoBILDA 5202/3/4 series)
    //    Servo port0: "rightRear led"
    //    I2C port0: "imu"          (REV internal IMU (BHI260AP))
    //    I2C port1: "pinpoint"     (GoBILDA Pinpoint Odometry Computer)

    // Declare motors
    private DcMotor _leftFront = null;
    private DcMotor _rightFront = null;
    private DcMotor _leftRear = null;
    private DcMotor _rightRear = null;

    private HardwareMap _hardwareMap;

    private double _lsy;
    private double _rsy;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        int wheelEncoderVal = 10;

        while (!isStopRequested()) {

            // Note: all wheels should rotate forward when left_stick_y is pushed up.
            // If any wheels rotate backwards then you have to revers the motor direction in
            // initRobot() and it's corresponding MecanumConstants.zzz parameters.

            double power = gamepad1.left_stick_y * 0.1;
            if (gamepad1.x) {
                _leftFront.setTargetPosition(_leftFront.getCurrentPosition() + wheelEncoderVal);
                _leftFront.setPower(power);
            }
            if (gamepad1.y) {
                _rightFront.setTargetPosition(_rightFront.getCurrentPosition() + wheelEncoderVal);
                _rightFront.setPower(power);
            }
            if (gamepad1.a) {
                _leftRear.setTargetPosition(_leftRear.getCurrentPosition() + wheelEncoderVal);
                _leftRear.setPower(power);
            }
            if (gamepad1.b) {
                _rightRear.setTargetPosition(_rightRear.getCurrentPosition() + wheelEncoderVal);
                _rightRear.setPower(power);
            }
        }

    }

    private void initRobot() throws InterruptedException {

        _hardwareMap = hardwareMap;

        // Make sure your ID's match your configuration
        _leftFront = _hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = _hardwareMap.get(DcMotor.class, "rightFront");
        _leftRear = _hardwareMap.get(DcMotor.class, "leftRear");
        _rightRear = _hardwareMap.get(DcMotor.class, "rightRear");

        // Set directions of the motors.
        // Left motors should be the opposite of the right motors.
        _rightFront.setDirection(DcMotorSimple.Direction.FORWARD);      // MecanumConstants.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        _rightRear.setDirection(DcMotorSimple.Direction.FORWARD);       // MecanumConstants.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        _leftFront.setDirection(DcMotorSimple.Direction.REVERSE);       // MecanumConstants.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        _leftRear.setDirection(DcMotorSimple.Direction.REVERSE);        // MecanumConstants.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    }

}