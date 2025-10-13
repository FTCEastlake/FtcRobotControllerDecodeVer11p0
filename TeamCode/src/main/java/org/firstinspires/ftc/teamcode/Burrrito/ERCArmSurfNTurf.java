package org.firstinspires.ftc.teamcode.Burrrito;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




/**
 * -----NOTES-----
 * all hardware maps for anything other than drivetrain is on expansion hub, not controller hub
 * you must set target position before setting the mode to RUN_TO_POSITION.
 * how to override:
 *      1. hold right bumper and return all motors to start position
 *      2. reset motor encoders by pressing "start"
 * _glbConfig.maxSliderEncoderVal will have to be variable due to the horizontal expansion limit (R104) 42 inch max
 *      Use trig for this?
 * reversing motors also reverses encoder readings
 * **/

public class ERCArmSurfNTurf {

    /**
     *Gamepad2:
     * Rotation:
     *   left stick Y: raise or lower arm rotation
     * Slider:
     *   Right stick Y: extend/retract slider
     *   Start Button: Reset slider encoder to 0
     *   Right Bumper: Override slider max/min, use when resetting encoder to 0
     * Claw:
     *   B: claw open
     *   A: claw close
     */

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private Gamepad _gamepad2;
    private ElapsedTime _runtime = new ElapsedTime();


    private DcMotor _slide = null;
    private DcMotor _rotation = null;
    private Servo _clawServo;

    private double maxClawPosition = 0.28;
    public final int _maxSliderEncoderVal = 3800; //max is about 4200
    public final int _minSliderEncoderVal = 50;

    public int _slideEncoderVal;
    public int _rotationEncoderVal;
    private int _lastRotationEncoderVal = _minSliderEncoderVal;//used to keep arm motor in place during weight shifting of slide
    private double _clawPosition;

    private double _lsy;
    private double _rsy;

    private final String _paramSlideEncoder = "Arm Slider Encoder";
    private final String _paramRotationEncoder= "Arm Rotation Encoder";
    private final String _paramClawPosition = "Claw Position";
//    private String _paramArmLsy = "Arm lsy";

    public ERCArmSurfNTurf(LinearOpMode opMode) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _gamepad2 = _opMode.gamepad2;

        init();
    }

    private void init() {

        _rotation = _hardwareMap.dcMotor.get("rotationMotor"); //--> motor port 0
        _rotation.setDirection(DcMotor.Direction.REVERSE);
        _rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _slide = _hardwareMap.dcMotor.get("slideMotor"); //--> motor port 1
        _rotation.setDirection(DcMotor.Direction.REVERSE);
        _slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetArmMotorEncoders();
        holdSlideMotor(0);//slide will fall forward in starting position

        _clawServo = _hardwareMap.get(Servo.class, "clawServo"); //--> servo port 0
        _clawServo.setPosition(maxClawPosition);
        _clawPosition = _clawServo.getPosition();
    }

    //main function that handles operator actions
    public void setArm() throws InterruptedException {

        _slideEncoderVal = _slide.getCurrentPosition();

        _lsy = -_gamepad2.left_stick_y;
        _rsy = _gamepad2.right_stick_y;

        //use variables to check values instead of calling .getPosition() multiple times per cycle
        _slideEncoderVal = _slide.getCurrentPosition();
        _rotationEncoderVal = _rotation.getCurrentPosition();
        _clawPosition = _clawServo.getPosition();

        if (!_gamepad2.right_bumper) {
            setSliderNormalMode(); //otherwise run normal actions
            setRotationNormalMode();
        } else {
            setSliderPower(_lsy); //right bumper runs override
            setRotationPower(_rsy);
        }

//        if (_gamepad2.dpad_left) { //note: this button has to be held to go to the specimen hang position
//            holdRotationMotor(-205);
//            holdSlideMotor(1100);
//        }
//
//        if (_gamepad2.dpad_right) {
//            holdRotationMotor(-2050);
//            holdSlideMotor(1050);
//        }

        if (_gamepad2.b) setClawOpen();
        if (_gamepad2.a) setClawClose();

        //log values
        DcMotor.RunMode runMode =  _slide.getMode();
    }

    //self explanatory
    //see note for resetting at top of this class
    public void resetArmMotorEncoders() {
        _slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        holdSlideMotor(_minSliderEncoderVal);
        _rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        holdRotationMotor(0,.25);
    }

    /**-----CLAW CONTROLS-----**/
    // set variables for max claw extensions in ERCGlobalConfig
    public void setClawOpen() {
        _clawServo.setPosition(0.0);
    }
    public void setClawClose() {
        _clawServo.setPosition(maxClawPosition);
    }

    /**-----SLIDE MOTOR CONTROLS-----**/
    //keeps slide steady at specified position
    public void holdSlideMotor(int position) {
        //keeps slide in place
        _slide.setTargetPosition(position);
        _slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _slide.setPower(0.05);
    }

    //use normal power (below) if in bounds, otherwise return to in bounds and hold position
    public void setSliderNormalMode() {
        if (_slideEncoderVal > _maxSliderEncoderVal) {
            holdSlideMotor(_maxSliderEncoderVal - 25); //+/- 25 in place to create a buffer zone and ensure that slide is returned to bounds
        } else if (_slideEncoderVal < _minSliderEncoderVal) {
            holdSlideMotor(_minSliderEncoderVal + 25);
        } else if (_lsy < -.1 || _lsy > .1) {
            setSliderPower(_lsy);
        } else {
            holdSlideMotor(_slideEncoderVal);
        }
    }

    //move slider to specified encoder value at specified speed
    //only use for autonomous actions
    public void setSliderEncoderAuto(int encoderVal, double power) {
        _runtime.reset();
        _slideEncoderVal = encoderVal;
        _slide.setTargetPosition(encoderVal);
        _slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _slide.setPower(Math.abs(power));    // don't have to set negative as setTargetPosition() is already inverted

        // Wait for slider to go to encoder value then shut off power to the motors so they won't overheat.
        while (_slide.isBusy() && !_gamepad2.right_bumper && _opMode.opModeIsActive() && (_runtime.seconds() <= 5)) {
            // Wait for completion
            //_logger.writeMsg("waiting for slide to reach position" + encoderVal);
        }
        holdSlideMotor(_slideEncoderVal);
        logSliderEncoderValues();
    }

    //move slider at specified power (doubles as override)
    public void setSliderPower(double upPower) {
        _slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _slide.setPower(upPower);
        logSliderEncoderValues();
    }

    //self explanatory
    private void logSliderEncoderValues() {
        _slideEncoderVal = _slide.getCurrentPosition();
    }

    /**-----ROTATION MOTOR CONTROLS-----**/
    //keeps Rotation rotation steady at specified encoder value
    public void holdRotationMotor(int position, double rotationPower) {
        //keeps Rotation in position
        _rotation.setTargetPosition(position);
        _rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rotation.setPower(Math.abs(rotationPower));
    }

    //moves rotation to a encoder position at specified power
    //only use for autonomous actions
    public void setRotationEncoderAuto(int encoderVal, double power) { //"0" on encoder is all the way down/front

        _rotationEncoderVal = encoderVal;
        _rotation.setTargetPosition(encoderVal);
        _rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rotation.setPower(Math.abs(power));

        while (_rotation.isBusy() && !_gamepad2.right_bumper) {
            //_logger.writeMsg("waiting for rotation to reach target" + encoderVal);
        }
        setRotationPower(0); // here so motor stops being busy when done
        logRotationEncoderValues();
    }

    //move rotation at controller right stick speed, if axis close to 0 hold the rotation steady
    public void setRotationNormalMode() {
        if (_rsy <= -.1 || _rsy >= .1) {
            setRotationPower(_rsy);
            _lastRotationEncoderVal = _rotationEncoderVal;
        } else {
            holdRotationMotor(_lastRotationEncoderVal,.2);
        }
    }

    public void setRotationPower(double power) {
        _rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rotation.setPower(power);
        logRotationEncoderValues();
    }

    //self explanatory
    private void logRotationEncoderValues() {
        _rotationEncoderVal = _rotation.getCurrentPosition();
    }
}