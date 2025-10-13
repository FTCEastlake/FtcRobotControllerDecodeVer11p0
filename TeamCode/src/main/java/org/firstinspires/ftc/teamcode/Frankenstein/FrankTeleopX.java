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
@TeleOp(name = "FrankTeleopX")
public class FrankTeleopX extends LinearOpMode {

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
    private static final double DESIRED_DISTANCE = 30.0; // 24 inches from tag

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
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


            // Automated PathFollowing
            if (gamepad1.aWasPressed())
            {
                Pose curPose = follower.getPose();

                // Create a new pose that moves 4 inches forward and turns 25 degrees CCW
                // Forward direction is based on the robot's current heading
                double forwardDistance = 15.0; // inches
                double turnAngle = Math.toRadians(25); // convert 25 degrees to radians (CCW is positive)

                // Calculate the new position
                double newX = curPose.getX() + forwardDistance * Math.cos(curPose.getHeading());
                double newY = curPose.getY() + forwardDistance * Math.sin(curPose.getHeading());
                double newHeading = curPose.getHeading() + turnAngle;

                telemetryM.addData("newHeading: ", newHeading);

                Pose targetPose = new Pose(newX, newY, newHeading);
                //AlignToAprilTag(30.0);
                pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                        .addPath(new Path(new BezierLine(curPose, targetPose)))
                        .setLinearHeadingInterpolation(curPose.getHeading(), targetPose.getHeading())
                        //.setConstantHeadingInterpolation(targetPose.getHeading())
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
            String formattedVelocity= String.format("magnitude=%.5f, theta=%.5f, xComponent=%.5f, yComponent=%.5f",
                    currentVelocity.getMagnitude(), currentVelocity.getTheta(), currentVelocity.getXComponent(), currentVelocity.getYComponent());
            telemetryM.addData("velocity", formattedVelocity);
            telemetryM.addData("automatedDrive", automatedDrive);

            _aprilTag = _vision.getFirstDetectedTag();
            if (_aprilTag != null)
            {
                telemetryM.addData("AprilTagX left(-), right(+) (inches)", _aprilTag.ftcPose.x);
                telemetryM.addData("AprilTagY front(+), back(-) (inches)", _aprilTag.ftcPose.y);
                telemetryM.addData("AprilTagRange (inches)", _aprilTag.ftcPose.range);
                telemetryM.addData("AprilTagYaw counter clockwise(+) (degrees)", _aprilTag.ftcPose.yaw);
                telemetryM.addData("AprilTagBearing counter clockwise(+) (degrees)", _aprilTag.ftcPose.bearing);
            }
            //telemetryM.update();
        }


    }

    /**
     * Normalizes angle to [-180, 180] degree range
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Normalizes angle to [-π, π] radian range
     */
    private static double normalizeAngleRadians(double angle) {
        double twoPI = 2.0 * Math.PI;
        double radianAngle = angle % twoPI;
        if (radianAngle > Math.PI) {
            radianAngle -= twoPI;
        } else if (radianAngle < -Math.PI) {
            radianAngle += twoPI;
        }
        return radianAngle;
    }

    private void initRobot() throws InterruptedException {
        follower = FrankConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        _vision = new Vision(this);

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
        follower.activateAllPIDFs();
        follower.setMaxPower(MAX_POWER);     // you can configure the power for each state
    }

    private void AlignToAprilTag() {

        //_aprilTag = _vision.getFirstDetectedTag();
        AprilTagDetection targetTag  = _vision.getFirstDetectedTag();
        if (targetTag  == null)
            return;

        Pose currentPose = follower.getPose();

        // Calculate target position relative to robot
        // AprilTag: range is forward distance, bearing is horizontal angle (+ left)
        // Adjust for desired distance (subtract to stay back from tag)
        double adjustedRange = targetTag.ftcPose.range - DESIRED_DISTANCE;

        // Calculate relative position in robot frame
        double forwardOffset = adjustedRange * Math.cos(Math.toRadians(targetTag.ftcPose.bearing));
        double leftOffset = adjustedRange * Math.sin(Math.toRadians(targetTag.ftcPose.bearing));

        // Convert robot heading to radians (Pedro uses degrees, but we need radians for trig)
        double currentHeadingRad = Math.toRadians(currentPose.getHeading());

        // Transform to field coordinates
        // Pedro: X is forward, Y is left
        double targetX = currentPose.getX() +
                (forwardOffset * Math.cos(currentHeadingRad) -
                        leftOffset * Math.sin(currentHeadingRad));
        double targetY = currentPose.getY() +
                (forwardOffset * Math.sin(currentHeadingRad) +
                        leftOffset * Math.cos(currentHeadingRad));

        // Calculate target heading (CCW positive in degrees)
        double targetHeading = currentPose.getHeading();

        // Normalize heading to -180 to 180
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;

        Pose targetPose = new Pose(targetX, targetY, targetHeading);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(currentPose, targetPose)))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//
//        // Calculate alignment errors
//        double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;     // inch
//        double headingError = -targetTag.ftcPose.bearing;   // Degrees. Negative for correction
//        double yawError = targetTag.ftcPose.yaw;            // Degrees
//
//
//
//        // Calculate target position relative to robot
//        // Range is forward distance, bearing is horizontal angle
//        double forward = targetTag.ftcPose.range * Math.cos(Math.toRadians(targetTag.ftcPose.bearing));
//        double strafe = targetTag.ftcPose.range * Math.sin(Math.toRadians(targetTag.ftcPose.bearing));
//
//        // Convert to field coordinates
//        Pose currentPose = follower.getPose();
//        double targetX = currentPose.getX() +
//                (forward * Math.cos(currentPose.getHeading()) -
//                        strafe * Math.sin(currentPose.getHeading()));
//        double targetY = currentPose.getY() +
//                (forward * Math.sin(currentPose.getHeading()) +
//                        strafe * Math.cos(currentPose.getHeading()));
//        double targetHeading = currentPose.getHeading() + Math.toRadians(headingError);
//
//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(currentPose, targetPose)))
//                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
//                .build();

    }

    private void AlignToAprilTag(double inchesBack) {

        //_aprilTag = _vision.getFirstDetectedTag();
        AprilTagDetection detection = _vision.getFirstDetectedTag();

        if (_aprilTag != null) {

            // Get current robot pose
            Pose currentPose = follower.getPose();

            // Calculate target pose based on AprilTag
            Pose targetPose = calculateTargetPose(currentPose, detection);

            // Create path to target pose
            Supplier<PathChain> alignmentPath = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(currentPose, targetPose)))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                    .build();

        }
    }

    /**
     * Calculate target pose to align with AprilTag
     * @param currentPose Current robot pose
     * @param detection AprilTag detection data
     * @return Target pose for alignment
     */
    private Pose calculateTargetPose(Pose currentPose, AprilTagDetection detection) {
        // Get AprilTag position relative to robot (from camera)
        double tagX = detection.ftcPose.x;
        double tagY = detection.ftcPose.y;
        double tagYaw = Math.toRadians(detection.ftcPose.yaw);

        // Convert tag position to field coordinates
        double robotHeading = currentPose.getHeading();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        // Tag position in field frame
        double tagFieldX = currentPose.getX() + (tagX * cosH - tagY * sinH);
        double tagFieldY = currentPose.getY() + (tagX * sinH + tagY * cosH);

        // Calculate angle to face the tag
        double angleToTag = Math.atan2(tagFieldY - currentPose.getY(),
                tagFieldX - currentPose.getX());

        // Calculate target position (maintain TARGET_DISTANCE from tag)
        double TARGET_DISTANCE = 30.0;
        double targetX = tagFieldX - TARGET_DISTANCE * Math.cos(angleToTag);
        double targetY = tagFieldY - TARGET_DISTANCE * Math.sin(angleToTag);

        // Target heading faces the tag
        double targetHeading = angleToTag;

        return new Pose(targetX, targetY, targetHeading);
    }

    /**
     * Calculate target pose based on AprilTag detection
     * This assumes you want to face the tag at a certain distance
     */
    private Pose calculateTargetPoseFromTag(AprilTagDetection tag, Pose currentPose) {
        // Get tag pose in robot frame
        double tagX = tag.ftcPose.x;
        double tagY = tag.ftcPose.y;
        double tagYaw = Math.toRadians(tag.ftcPose.yaw);

        // Transform to field frame
        double robotHeading = currentPose.getHeading();
        double fieldTagX = currentPose.getX() + tagX * Math.cos(robotHeading) - tagY * Math.sin(robotHeading);
        double fieldTagY = currentPose.getY() + tagX * Math.sin(robotHeading) + tagY * Math.cos(robotHeading);

        // Calculate desired position (DESIRED_DISTANCE away from tag, facing it)
        double angleToTag = Math.atan2(fieldTagY - currentPose.getY(), fieldTagX - currentPose.getX());

        double targetX = fieldTagX - DESIRED_DISTANCE * Math.cos(angleToTag);
        double targetY = fieldTagY - DESIRED_DISTANCE * Math.sin(angleToTag);
        double targetHeading = angleToTag; // Face the tag

        return new Pose(targetX, targetY, targetHeading);
    }


    /**
     * Calculate target pose based on AprilTag detection
     * This assumes you want to face the tag at a certain distance
     */
//    private Pose calculateTargetPoseFromTag(AprilTagDetection tag, Pose currentPose) {
//        // Get tag pose in robot frame
//        double tagX = tag.ftcPose.x;
//        double tagY = tag.ftcPose.y;
//        double tagYaw = Math.toRadians(tag.ftcPose.yaw);
//
//        // Transform to field frame
//        double robotHeading = currentPose.getHeading();
//        double fieldTagX = currentPose.getX() + tagX * Math.cos(robotHeading) - tagY * Math.sin(robotHeading);
//        double fieldTagY = currentPose.getY() + tagX * Math.sin(robotHeading) + tagY * Math.cos(robotHeading);
//
//        // Calculate desired position (DESIRED_DISTANCE away from tag, facing it)
//        double angleToTag = Math.atan2(fieldTagY - currentPose.getY(), fieldTagX - currentPose.getX());
//
//        double targetX = fieldTagX - DESIRED_DISTANCE * Math.cos(angleToTag);
//        double targetY = fieldTagY - DESIRED_DISTANCE * Math.sin(angleToTag);
//        double targetHeading = angleToTag; // Face the tag
//
//        return new Pose(targetX, targetY, targetHeading);
//    }

}