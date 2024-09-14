package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class DetectionPipeline extends OpenCvPipeline {

    public static double THRESH = 5;
    public enum POSITION {
        LEFT,
        MID,
        RIGHT
    }
    static final Scalar RED = new Scalar(255, 0, 0);
    public static String leftPosValue;
    public static String midPosValue;
    public static String rightPosValue;

    public static int LEFT_CORNER_X = 20;
    public static int LEFT_CORNER_Y = 600;
    public static int MID_CORNER_X = 580;
    public static int MID_CORNER_Y = 550;
    public static int RIGHT_CORNER_X = 1165;
    public static int RIGHT_CORNER_Y = 650;

    static Point MID_CORNER;
    static Point LEFT_CORNER;
    static Point RIGHT_CORNER;

    public static int rectWidth = 100; // Define your own rectangle width
    public static int rectHeight = 100; // Define your own rectangle height

    private volatile POSITION position = POSITION.LEFT;

    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat combinedResult = new Mat();
    Mat gray = new Mat();
    Mat binary = new Mat();

    Mat midRegion;
    Mat leftRegion;
    Mat rightRegion;

    public DetectionPipeline() {
        MID_CORNER = new Point(MID_CORNER_X,MID_CORNER_Y);
        LEFT_CORNER = new Point(LEFT_CORNER_X,LEFT_CORNER_Y);
        RIGHT_CORNER = new Point(RIGHT_CORNER_X,RIGHT_CORNER_Y);
//        gaus = new Mat();
//        hsv = new Mat();
//        mask = new Mat();
//        combinedResult = new Mat();
//        gray = new Mat();
//        binary = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
//        hsv = new Mat();
//        mask = new Mat();
//        combinedResult = new Mat();
//        gray = new Mat();
//        binary = new Mat();

//        Imgproc.GaussianBlur(input, gaus, new Size(GAUS, GAUS), 0);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar lower = new Scalar(0, 150, 0);
        Scalar upper = new Scalar(255, 255, 255);
        Core.inRange(hsv, lower, upper, mask);

        Core.bitwise_and(input, input, combinedResult, mask);

        Imgproc.cvtColor(combinedResult, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply thresholding to convert to black and white
        Imgproc.threshold(gray, binary, THRESH, 255, Imgproc.THRESH_BINARY);

        // Extract the region of interest (ROI) based on the rectangle
        Rect mid = new Rect((int) MID_CORNER.x, (int) MID_CORNER.y, rectWidth, rectHeight);
        Rect left = new Rect((int) LEFT_CORNER.x, (int) LEFT_CORNER.y, rectWidth, rectHeight);
        Rect right = new Rect((int) RIGHT_CORNER.x, (int) RIGHT_CORNER.y, rectWidth, rectHeight);

        midRegion = binary.submat(mid);
        leftRegion = binary.submat(left);
        rightRegion = binary.submat(right);

        Imgproc.rectangle(
                binary,
                LEFT_CORNER,
                sizedCorner(LEFT_CORNER),
                RED,
                2);

        Imgproc.rectangle(
                binary,
                MID_CORNER,
                sizedCorner(MID_CORNER),
                RED,
                2);

        Imgproc.rectangle(
                binary,
                RIGHT_CORNER,
                sizedCorner(RIGHT_CORNER),
                RED,
                2);

        // Calculate the average color of the rectangle
        Scalar midColor = Core.mean(midRegion);
        Scalar leftColor = Core.mean(leftRegion);
        Scalar rightColor = Core.mean(rightRegion);

        // Output the average BGR color values
        double[] leftBGR = leftColor.val;
        double[] midBGR = midColor.val;
        double[] rightBGR = rightColor.val;
        this.leftPosValue = "Average R Color: " + leftBGR[0] + "Average G Color: " + leftBGR[1] + "Average B Color: " + leftBGR[2];
        this.midPosValue = "Average R Color: " + midBGR[0] + "Average G Color: " + midBGR[1] + "Average B Color: " + midBGR[2];
        this.rightPosValue = "Average R Color: " + rightBGR[0] + "Average G Color: " + rightBGR[1] + "Average B Color: " + rightBGR[2];

        // Draw the rectangle on the frame for visualization
        Imgproc.rectangle(binary, LEFT_CORNER, sizedCorner(LEFT_CORNER), new Scalar(0, 255, 0), 10);
        Imgproc.rectangle(binary, MID_CORNER, sizedCorner(MID_CORNER), new Scalar(0, 255, 0), 10);
        Imgproc.rectangle(binary, RIGHT_CORNER, sizedCorner(RIGHT_CORNER), new Scalar(0, 255, 0), 10);

        if (leftBGR[0] > midBGR[0] && leftBGR[0] > rightBGR[0]) {
            // LEFT
            position = POSITION.LEFT;
            Imgproc.rectangle(binary, LEFT_CORNER, sizedCorner(LEFT_CORNER), new Scalar(0, 255, 0), -1);
        } else if (midBGR[0] > leftBGR[0] && midBGR[0] > rightBGR[0]) {
            // MID
            position = POSITION.MID;
            Imgproc.rectangle(binary, MID_CORNER, sizedCorner(MID_CORNER), new Scalar(0, 255, 0), -1);
        } else {
            // RIGHT
            position = POSITION.RIGHT;
            Imgproc.rectangle(binary, RIGHT_CORNER, sizedCorner(RIGHT_CORNER), new Scalar(0, 255, 0), -1);
        }

        hsv.release();
        mask.release();
        combinedResult.release();
        gray.release();
        midRegion.release();
        leftRegion.release();
        rightRegion.release();

        return binary;
    }

    public POSITION getPos() {
        return this.position;
    }

    public String getLeftBoxVal() {
        return leftPosValue;
    }
    public String getMidBoxVal() {
        return midPosValue;
    }
    public String getRightBoxVal() {
        return rightPosValue;
    }


    public Point sizedCorner(Point corner) {
        return new Point(corner.x + rectWidth, corner.y + rectHeight);
    }
}