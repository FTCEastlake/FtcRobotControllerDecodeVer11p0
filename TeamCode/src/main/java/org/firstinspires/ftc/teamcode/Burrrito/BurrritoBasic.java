package org.firstinspires.ftc.teamcode.Burrrito;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.ColorSensorV3;

@Disabled
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

    private GoBildaPinpointDriver _pinpoint;

    private ColorSensorV3 _colorSensor = null;
    private ColorSensorV3.DetectedColor _detectedColor;

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

        String pinpointInfo1 = "Push your robot around to see it track";
        String pinpointInfo2 = "Press left bumper to reset the pinpoint position";
        String pinpointInfo3 = "Pinpoint X and Y values should be positive forward and left, respectively";
        String pinpointInfo4 = "Pinpoint heading angle (degrees) should be positive counter-clockwise";

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


            if(gamepad1.left_bumper){
                // You could use readings from April Tags here to give a new known position to the pinpoint
                _pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            _pinpoint.update();
            Pose2D pose2D = _pinpoint.getPosition();


            telemetry.addLine(powerInfo);
            telemetry.addLine(buttonInfo1);
            telemetry.addLine(buttonInfo2);
            telemetry.addLine(pinpointInfo1);
            telemetry.addLine(pinpointInfo2);
            telemetry.addLine(pinpointInfo3);
            telemetry.addLine(pinpointInfo4);
            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));


            _telemetryM.addLine(powerInfo);
            _telemetryM.addLine(buttonInfo1);
            _telemetryM.addLine(buttonInfo2);
            _telemetryM.addLine(pinpointInfo1);
            _telemetryM.addLine(pinpointInfo2);
            _telemetryM.addLine(pinpointInfo3);
            _telemetryM.addLine(pinpointInfo4);
            _telemetryM.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            _telemetryM.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            _telemetryM.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

            //_colorSensor.displayColorValues();
            //_detectedColor = _colorSensor.getDetectedColor();

            telemetry.update();
            _telemetryM.update();
        }

    }

    private void initRobot() throws InterruptedException {

        _hardwareMap = hardwareMap;
        _telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //_colorSensor = new ColorSensorV3(this, telemetry);

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

        // Gobilda pinpoint initialization
        initPinpoint();

    }

    private void initPinpoint() {

        _pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        // xOffset is inches the forward pod is left(+) or right(-) of center.
        // yOffset is inches the strafe pod is behind(-) of front of robot.
        _pinpoint.setOffsets(-2.3125, -9.5, DistanceUnit.INCH);


        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        _pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        _pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        _pinpoint.resetPosAndIMU();

        // Set the location of the robot - this should be the place you are starting the robot from
        _pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

}