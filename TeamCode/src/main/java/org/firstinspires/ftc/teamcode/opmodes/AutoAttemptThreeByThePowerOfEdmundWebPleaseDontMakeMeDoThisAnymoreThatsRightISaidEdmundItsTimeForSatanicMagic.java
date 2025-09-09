package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class AutoAttemptThreeByThePowerOfEdmundWebPleaseDontMakeMeDoThisAnymoreThatsRightISaidEdmundItsTimeForSatanicMagic
implements VisionProcessor{
    public Rect rect = new Rect(20,20,50,50);
    @Override
    public void init(int width, int height, CameraCalibration calibration){

    }
    public Object processFrame(Mat Frame, long captureVoidNanos){
        return null;
    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx ){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
         int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
         int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userDensity){

    }
}

