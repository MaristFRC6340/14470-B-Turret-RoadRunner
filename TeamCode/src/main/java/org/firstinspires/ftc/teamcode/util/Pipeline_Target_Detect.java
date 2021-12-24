package org.firstinspires.ftc.teamcode.util;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.improc.Improc;
import org.opencv.imgcodescs.Imgcodecs;
import org.opencv.core.CvType;
import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;

public class Pipeline_Target_Detect extends openCvPipeline {

    //Create the template Object (populated in constructor)
    Mat templateMat = new Mat(64, 64, CvType.CV_8UC1);

    //Stores the x position of found target (not found = -1)
    private double xPos = -1;

    public Pipeline_Target_Detect() {
        buildTemplate();
    }

    //Getter for xPos
    public double getXPos() {
        return xPos;
    }

    @Override
    public Mat processFrame(Mat input) {    //"Mat input" represents image from camera

        //Convert to greyscale
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        //Template Match
        Mat heatMap = new Mat();
        int macMethod = Imgproc.TM_CCOEFF;
        Imgproc.matchTemplate(gray, templateMat, heatMap, machMethod);

        //Find the coordinates of the best match
        MinMaxLocResult mmr = Core.minMaxLoc(heatMap);
        Point matchLoc = mmr.maxLoc;

        //Draw rectangle around the best match
        Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x+templateMat.cols(),
                mathcLoc.y+temoplateMat.rows()), new Scalar(0, 255, 0));

        //Assign to xPos
        xPos = (double)matchLoc.x;

        return input;
    }

    private void buildTemplate() {
        int[][] template = new int[64][64];

        for(int r = 0; r < 32; r++) {
            for(int c = 0; c < 32; c++) {
                template[r][c] = 255;
            }
        }

        for(int r = 32; r<64; r++) {
            for(int c = 32; c<64; c++) {
                template[r][c] = 255;
            }
        }

        Mat templateMat = new Mat(64, 64, CvType.CV_8UC1);
        for(int r = 0; r<64; r++) {
            for(int c = 0; c<64; c++) {
                templateMat.put(r, c, template[r][c]);
            }
        }
    }


}
