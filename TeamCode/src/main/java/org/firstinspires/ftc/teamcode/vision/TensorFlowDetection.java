package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TensorFlowDetection {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    //private static String TFOD_MODEL_ASSET = "model_20231127_143238.tflite"; //model without negatives
    //private static String TFOD_MODEL_ASSET = "model_20231209_095349.tflite"; //model with negatives
    //private static String TFOD_MODEL_ASSET = "model_20240309_150637.tflite";
    private static String TFOD_MODEL_ASSET = "goodModel.tflite";
    private static final String[] LABELS = {
            "prop",
    };
    public TensorFlowDetection(WebcamName camera) {


        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//                //   .setModelAssetName("model_20231104_124524.tflite")
//
//                // Use setModelAssetName() if the TF Model is built in as an asset.
//                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                .setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                .setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                //.setModelAspectRatio(16.0 / 9.0)
//
//                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(camera);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        //builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }// end method initTfod()
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }   // end for() loop
//
//
//    }   // end method telemetryTfod()

    public enum PropPosition{
        LEFT, RIGHT, CENTER
    }



    public PropPosition getPropPosition(Timeout timeout) {
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        while(currentRecognitions.size() < 1 && !timeout.expired()) {
//            //android.util.Log.w("TENSORFLOW", "Spinning for current recognitions...");
//            currentRecognitions = tfod.getRecognitions();
//        }
//        if (currentRecognitions.size() < 1) {
//            return null;
//        }
//        Recognition recognition = currentRecognitions.get(0);
//        while (recognition.getWidth() > 300 && !GlobalOpMode.opMode.isStopRequested()) {
//            android.util.Log.w("TENSORFLOW", "Spinning for current recognitions...");
//            for (Recognition rec :currentRecognitions) {
//                if (rec.getWidth() < recognition.getWidth()) {
//                    recognition = rec;
//                }
//            }

        // GET MORE CURRENT RECOGNITIONS?
//        }
        //visionPortal.close();



//        double centerX = (recognition.getLeft() + recognition.getRight()) / 2 ;
//        if (centerX < 214) {
//            //android.util.Log.w("TENSORFLOW", "LEFT");
//            return PropPosition.LEFT;
//        } else if(centerX > 214 && centerX < 428) {
//            //android.util.Log.w("TENSORFLOW", "CENTER");
//            return PropPosition.CENTER;
//        } else {
//            //android.util.Log.w("TENSORFLOW", "RIGHT");
//            return PropPosition.RIGHT;
//        }


        return null;
    }

}
