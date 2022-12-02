/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "redcentercamauto", group = "Concept")

public class TensorFlowOpMode_Copy extends LinearOpMode {
  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */
    private DcMotor raw_shoulder;
    private DcMotor raw_elbow;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private Servo LeftClaw;
    private Servo RightClaw;
    private Servo wrist;
    
    private MyMotor shoulder;
    private MyMotor elbow;
    
    String armStage;
    boolean first;
    int s = 0;
    int e = -7;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ATOG5mj/////AAABmfmXvSzj2Ux+pmz1IJNzKHF0VpNa0OnJxDLE2SeGlIGgWZr7F8iunY9sSFkGQd5Fi0XnRA5EaTv8iWrUgNp/dPZ15ZIk4O40kX5EtRMFrb3+mmAY/EkAzzRRnKHclpP7rH0S9qeQCLdEwb+WIbjscfqoHVaUlXkAV+/rPlfCC3PwOa7+iSwtKjij8W2ImpPwoYdXEr2yaizyS9PKsZ+yVQXulZCtNkTJUwBaujmODqSvpPKMa/W9T/oeiyiwj7nHqdfqIsDnA1gf1D4OmXri3BbJE89bJXUBWLgnu8sixs7ZsNWcXwookmME672KIx8JTIiLSJHE4ZepzbSLR7kU+DS1shBDtgDjQwFqJ62F72Nw";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.3, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        int loop = 0;
        raw_shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        raw_elbow = hardwareMap.get(DcMotor.class, "elbow");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        
        first = true;
        int clawMode = 0;
        int wristMode = 0;
        int clawLoop = 0;
        int wristLoop = 0;
        
        // Reverse one of the drive motors.
        raw_shoulder.setPower(0);
        raw_elbow.setPower(0);
        //raw_wrist.setPosition(.7);
        //raw_claw.setPosition(.25);
        shoulder = new MyMotor(raw_shoulder, -3350, 90, "raw_shoulder");
        elbow = new MyMotor(raw_elbow, -5000, 180, "raw_elbow");
        
        raw_elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raw_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raw_elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raw_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raw_elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        raw_shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        LeftClaw.setPosition(0);
        RightClaw.setPosition(.5);
        double BackPower = 0.35;
        double BackDuration = 0.5;
        String e;
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()){
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "marker") {
                          
                        } else {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addLine("%i");
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        }
                        if (recognition.getLabel() == "1 Bolt" || recognition.getLabel() == "2 Bulb") {
                            shoulder.goto_pos_degree(-50+s);
                            elbow.goto_pos_degree(-12);
                            int g;
                            int j;
                            int k;
                            sleep(1000);
                            elbow.goto_pos_degree(-2);
                            LeftClaw.setPosition(.25);
                            RightClaw.setPosition(.25);
                            sleep(1000);
                            if (recognition.getLabel() == "1 Bolt") {
                                g = 3000;
                                j = 1500;
                                k = 0;
                            } else {
                                g = 1000;
                                j = 3000;
                                k = 1500;
                
                            }
                            ElapsedTime f;
                                    f = new ElapsedTime();//left
                                    while (f.milliseconds() < g) {
                                        FrontLeft.setPower(-BackPower);
                                        FrontRight.setPower(BackPower);
                                        BackLeft.setPower(BackPower);
                                        BackRight.setPower(-BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    f = new ElapsedTime();
                                    while (f.milliseconds() < j) {
                                        FrontLeft.setPower(-BackPower);
                                        FrontRight.setPower(BackPower);
                                        BackLeft.setPower(-BackPower);
                                        BackRight.setPower(BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    
                                    f = new ElapsedTime();
                                    while (f.milliseconds() < k) {
                                        FrontLeft.setPower(BackPower);
                                        FrontRight.setPower(-BackPower);
                                        BackLeft.setPower(BackPower);
                                        BackRight.setPower(-BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    shoulder.goto_pos_degree(0);
                                    elbow.goto_pos_degree(5);
                                    
                                    shoulder.goto_pos_degree(0);
                                    elbow.goto_pos_degree(0);
                                    break;
                            } else if (recognition.getLabel() == "3 Panel") {
                                shoulder.goto_pos_degree(-50+s);
                                elbow.goto_pos_degree(-12);
                                int g;
                                sleep(1000);
                                elbow.goto_pos_degree(-2);
                                LeftClaw.setPosition(.25);
                                RightClaw.setPosition(.25);
                                sleep(1000);
                                ElapsedTime f;
                                    f = new ElapsedTime();//left
                                    while (f.milliseconds() < 1000) {
                                        FrontLeft.setPower(BackPower);
                                        FrontRight.setPower(-BackPower);
                                        BackLeft.setPower(-BackPower);
                                        BackRight.setPower(BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    f = new ElapsedTime();
                                    while (f.milliseconds() < 1500) {
                                        FrontLeft.setPower(-BackPower);
                                        FrontRight.setPower(BackPower);
                                        BackLeft.setPower(-BackPower);
                                        BackRight.setPower(BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    shoulder.goto_pos_degree(0);
                                    elbow.goto_pos_degree(0);
                                    break;
                            }
                        }
                        telemetry.update();
                        break;
                      }
                      break;
                      /*telemetry.update();
                      loop += 1;
                      e = "hi";
                      if (loop > 1000) {
                            shoulder.goto_pos_degree(-50+s);
                           // elbow.goto_pos_degree(-5+e);
                            
                            sleep(1000);
                            //elbow.goto_pos_degree(5+e);
                            LeftClaw.setPosition(.25);
                            RightClaw.setPosition(.25);
                            sleep(1000);
                            ElapsedTime f;
                            if (e == "") {
                                    f = new ElapsedTime();//left
                                    while (f.milliseconds() < 3000) {
                                        FrontLeft.setPower(-BackPower);
                                        FrontRight.setPower(BackPower);
                                        BackLeft.setPower(BackPower);
                                        BackRight.setPower(-BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    f = new ElapsedTime();
                                    while (f.milliseconds() < 500) {
                                        FrontLeft.setPower(-BackPower);
                                        FrontRight.setPower(BackPower);
                                        BackLeft.setPower(-BackPower);
                                        BackRight.setPower(BackPower);
                                    }
                                    FrontLeft.setPower(0);
                                    FrontRight.setPower(0);
                                    BackLeft.setPower(0);
                                    BackRight.setPower(0);
                                    shoulder.goto_pos_degree(0);
                                    elbow.goto_pos_degree(5);
                            }
                            
                            f = new ElapsedTime();//left
                            while (f.milliseconds() < 3000) {
                                FrontLeft.setPower(-BackPower);
                                FrontRight.setPower(BackPower);
                                BackLeft.setPower(BackPower);
                                BackRight.setPower(-BackPower);
                            }
                            FrontLeft.setPower(0);
                            FrontRight.setPower(0);
                            BackLeft.setPower(0);
                            BackRight.setPower(0);
                            f = new ElapsedTime();
                            while (f.milliseconds() < 500) {
                                FrontLeft.setPower(BackPower);
                                FrontRight.setPower(-BackPower);
                                BackLeft.setPower(BackPower);
                                BackRight.setPower(-BackPower);
                            }
                            FrontLeft.setPower(0);
                            FrontRight.setPower(0);
                            BackLeft.setPower(0);
                            BackRight.setPower(0);
                            shoulder.goto_pos_degree(0);
                            elbow.goto_pos_degree(5);
                    }*/
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.7f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
