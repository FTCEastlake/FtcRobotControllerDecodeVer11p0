package org.firstinspires.ftc.teamcode.Common;

import android.util.Size;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;
import java.util.function.Supplier;

public class Vision {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;

    private VisionPortal _visionPortal = null;               // Used to manage the video source.
    private AprilTagProcessor _aprilTag = null;              // Used for managing the AprilTag detection process.

    public Vision(LinearOpMode opMode) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;

        boolean includeAprilTagDetection =  true;
        init(includeAprilTagDetection);
    }

    private void init(boolean enableAprilTagDetection)
    {
        boolean useGoBuildaCamera  = true;
        if (enableAprilTagDetection)
        {
            // Create the AprilTag processor by using a builder.
            if (useGoBuildaCamera)
                _aprilTag = new AprilTagProcessor.Builder()
                        .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                        .build();
            else
                _aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // e.g. Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            _aprilTag.setDecimation(2);
        }



        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(_hardwareMap.get(WebcamName.class, "Webcam Front"));
        if (useGoBuildaCamera)
            builder.setCameraResolution(new Size(640,480));
        if (enableAprilTagDetection)
        {
            builder.addProcessor(_aprilTag);
        }
        _visionPortal = builder.build();
    }

    public AprilTagDetection getFirstDetectedTag() {

        List<AprilTagDetection> currentDetections = _aprilTag.getDetections();
        if (currentDetections.isEmpty())
            return null;

        return currentDetections.get(0);
    }

    public Pose AlignToAprilTag(Pose currentPose, double inchesBack) {

        //_aprilTag = _vision.getFirstDetectedTag();
        AprilTagDetection detectedTag  = getFirstDetectedTag();
        if (detectedTag  == null)
            return null;

        //Pose currentPose = follower.getPose();

        // Get AprilTag pose data
        double range = detectedTag.ftcPose.range;       // inches
        double bearing = detectedTag.ftcPose.bearing;   // degrees
        double yaw = detectedTag.ftcPose.yaw;           // degrees

        // Calculate distance to move (subtract desired stopping distance)
        double distanceToMove = range - inchesBack;

        // Calculate the angle to the tag in robot's reference frame
        double angleToTag = Math.toRadians(bearing);

        // Calculate the left and forward movement in the robot coordinate system.
        // Note: pedro pose.y is left(+)/right(-). April tag ftcPose.x is left(-)/right(+).
        // Note: pedro pose.x is forward(+). April tag ftcPose.y is forward(+).
        // Note: detectedTag.bearing is (=) counter clockwise.
        double targetPoseLeft = currentPose.getY() + distanceToMove * Math.sin(angleToTag);
        double targetPoseForward = currentPose.getX() + distanceToMove * Math.cos(angleToTag);

        // Calculate desired heading change
        // If you want to face the tag, use bearing
        // If you want to align with the tag's orientation, use yaw
        double headingChange = angleToTag; // To face the tag
        double newHeading = currentPose.getHeading() + headingChange;   // radians

        // Normalize heading to -180 to 180
        newHeading = normalizeAngleRadians(newHeading);

        // Pose targetPose = new Pose(newX, newY, newHeading);
        Pose targetPose = new Pose(targetPoseForward, targetPoseLeft, newHeading);
//        telemetryM.addData("targetPoseForward: ", targetPoseForward);
//        telemetryM.addData("targetPoseLeft: ", targetPoseLeft);
//        telemetryM.addData("newHeading (radians): ", newHeading);
//        telemetryM.addData("newHeading (degrees): ", Math.toDegrees(newHeading));

        return targetPose;
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

}
