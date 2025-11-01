package org.firstinspires.ftc.teamcode.Burrrito;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private TelemetryManager _telemetryM;

    private double _lsy;
    private double _rsy;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        int wheelEncoderVal = 10;

        String powerInfo = "Use left stick y for power";
        String buttonInfo1 = "Buttons: X = Left Front, Y = Right Front";
        String buttonInfo2 = "Buttons: A = Left Rear,  Y = Right Rear";

        telemetry.addLine(powerInfo);
        telemetry.addLine(buttonInfo1);
        telemetry.addLine(buttonInfo2);
        telemetry.update();

        _telemetryM.addLine(powerInfo);
        _telemetryM.addLine(buttonInfo1);
        _telemetryM.addLine(buttonInfo2);
        _telemetryM.update();

        while (!isStopRequested()) {

            // Note: all wheels should rotate forward when left_stick_y is pushed up.
            // If any wheels rotate backwards then you have to revers the motor direction in
            // initRobot() and it's corresponding MecanumConstants.zzz parameters.

            double power = gamepad1.left_stick_y * 0.25;
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
        _telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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