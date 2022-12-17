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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
@Autonomous(name = "PowerPlayRedCAMWMRMAM", group = "Concept")

public class PowerPlayRedAuto_Copy extends LinearOpMode {
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
   private static final double SHOULDER_POWER = 1;
  private static final double ELBOW_POWER = 0.6;
  private static final int SLOW_DOWN_THRESHOLD = 300;
  private static final double SLOW_DOWN_POWER = 0.3;
  private static final long DEBOUNCE_MS = 50;
  
  private static final double CLAW_CLOSE = 0.42;
  private static final double CLAW_OPEN = 0;
  
  private static final double WRIST_UP = 0;
  private static final double WRIST_DOWN = 1;
  
  private boolean hold = false;

  private DcMotor BackRight = null;
  private DcMotor BackLeft = null;
  private DcMotor FrontLeft = null;
  private DcMotor FrontRight = null;
  
  private DcMotor shoulder = null;
  private DcMotor elbow = null;
  private Servo clawLeft = null;
  private Servo clawRight = null;
  private Servo wrist = null;

  private enum ArmState {
    INIT,
    PICK1,
    PICK2,
    SHORT_POLE1,
    SHORT_POLE2,
    MIDDLE_POLE1,
    MIDDLE_POLE2,
    TALL_POLE1,
    TALL_POLE2
  };

  // variables for arm
  private ArmState armState;
  private int shoulderPosition;
  private int elbowPosition;
  private double wristPosition;
  private double clawPosition;
  private double shoulderDelay;
  private double elbowDelay;
  private double wristDelay;

  // controller
  private ElapsedTime timer;
  private boolean lastState;
  private boolean lastSteadyState;
  private double lastStateChangeTime;
  private double lastRequestTime;
  
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
            tfod.setZoom(1.3, 13.0/7.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        boolean wait = true;
        int box = 0;
        while (wait == true) {
          if (opModeIsActive()) {
            wait = false;
          }
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
                        
                        telemetry.addData(String.format("label "), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLabel().contains("2")) {
                          box = 2;
                        } else if (recognition.getLabel().contains("1")) {
                          box = 1;
                        } else if (recognition.getLabel().contains("3")) {
                          box = 3;
                        }
                        telemetry.addData("mogus", box);
                        i++;
                        }
                      }
                      telemetry.update();
                    }
                }
                
        }
        
        waitForStart();
        if (opModeIsActive()) {
          initHardwareMap();
          
          initHardwareConfig();
            
                clawPosition = CLAW_CLOSE;
                setArmPosition(0, 0, 0, 0, WRIST_DOWN, 0);
                moveArm();
                Right(2600);
                Forward(100);//150
                TurnLeft(50);
                setArmPosition(180, 0, -75, 0, WRIST_DOWN, 0); //200 500
                moveArm();
                Right(830);
                Left(10);
                //orward(2300);
                //TurnLeft(725);
                sleep(2000);
                clawPosition = CLAW_OPEN;
                setArmPosition(45, 0, -15, 0, WRIST_DOWN, 0);
                moveArm();
                sleep(500);
                
                Left(665);
                Right(10);
                setArmPosition(43, 0, -82, 0, WRIST_UP, 0);
                moveArm();
                sleep(500);
                Forward(675);
                sleep(200);
                clawPosition = CLAW_CLOSE;
                moveArm();
                sleep(300);
                setArmPosition(90, 0, -82, 0, WRIST_UP, 0); //200 500
                moveArm();
                sleep(100);
                Backward(460);//325
                sleep(200);
                Right(700);
                
                
                TurnLeft(75);
                
                setArmPosition(180, 0, -75, 0, WRIST_DOWN, 0); //200 500
                moveArm();
                sleep(1500);
                clawPosition = CLAW_OPEN;
                setArmPosition(45, 0, -15, 0, WRIST_DOWN, 0); //200 500
                moveArm();
                sleep(750);
                
                Left(750);
                
                TurnLeft(20);
                Right(11);
                setArmPosition(40, 0, -82, 0, WRIST_UP, 0);
                moveArm();
                sleep(500);
                Forward(725);
                clawPosition = CLAW_CLOSE;
                moveArm();
                sleep(300);
                setArmPosition(90, 0, -82, 0, WRIST_UP, 0); //200 500
                moveArm();
                sleep(100);
                Backward(500); //32
                sleep(200);
                Right(800);
                sleep(1000);
                
                setArmPosition(180, 0, -75, 0, WRIST_DOWN, 0); //200 500
                moveArm();
                sleep(1000);
                clawPosition = CLAW_OPEN;
                setArmPosition(45, 0, -15, 0, WRIST_DOWN, 0); //200 500
                moveArm();
                sleep(600);
                Left(2000);
                
                setArmPosition(0, 0, 0, 0, WRIST_DOWN, 0);
                moveArm();
                sleep(1000);
                if (box == 1) {
                  Forward(1000);
                } else if (box == 2) {
                } else if (box == 3) {
                  Backward(850);
                }
                
            }
        }
    
private void resetEncoders() {
      BackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
      BackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
      FrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
      FrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }
    private void runToPos() {
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void testMove() {
      resetEncoders();
      BackLeft.setPower(0.2);
      BackLeft.setTargetPosition(-1000);
      while (BackLeft.isBusy());
      BackLeft.setPower(0);
      sleep(5000);
    }
    double power2 = .8;
    private void Forward(int pos) {
      resetEncoders();
      FrontLeft.setPower(-power2);
      FrontLeft.setTargetPosition(-pos);
      FrontRight.setPower(power2);
      FrontRight.setTargetPosition(pos);
      BackLeft.setPower(-power2);
      BackLeft.setTargetPosition(-pos);
      BackRight.setPower(power2);
      BackRight.setTargetPosition(pos);
      runToPos();
      while (FrontLeft.isBusy());
      while (FrontRight.isBusy());
      while (BackLeft.isBusy());
      while (BackRight.isBusy());
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    } 
    private void Right(int pos) {
      resetEncoders();
      FrontLeft.setPower(-power2);
      FrontLeft.setTargetPosition(-pos);
      FrontRight.setPower(-power2);
      FrontRight.setTargetPosition(-pos);
      BackLeft.setPower(power2);
      BackLeft.setTargetPosition(pos);
      BackRight.setPower(power2);
      BackRight.setTargetPosition(pos);
      runToPos();
      while (FrontLeft.isBusy());
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
    private void Left(int pos) {
      resetEncoders();
      FrontLeft.setPower(power2);
      FrontLeft.setTargetPosition(pos);
      FrontRight.setPower(power2);
      FrontRight.setTargetPosition(pos);
      BackLeft.setPower(-power2);
      BackLeft.setTargetPosition(-pos);
      BackRight.setPower(-power2);
      BackRight.setTargetPosition(-pos);
      runToPos();
      while (FrontLeft.isBusy());
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
    private void Backward(int pos) {
      resetEncoders();
      FrontLeft.setPower(power2);
      FrontLeft.setTargetPosition(pos);
      FrontRight.setPower(-power2);
      FrontRight.setTargetPosition(-pos);
      BackLeft.setPower(power2);
      BackLeft.setTargetPosition(pos);
      BackRight.setPower(-power2);
      BackRight.setTargetPosition(-pos);
      runToPos();
      while (FrontLeft.isBusy());
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
    private void TurnRight(int pos) {
      resetEncoders();
      FrontLeft.setPower(-power2);
      FrontLeft.setTargetPosition(-pos);
      FrontRight.setPower(-power2);
      FrontRight.setTargetPosition(-pos);
      BackLeft.setPower(-power2);
      BackLeft.setTargetPosition(-pos);
      BackRight.setPower(-power2);
      BackRight.setTargetPosition(-pos);
      runToPos();
      while (FrontLeft.isBusy());
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
      
    }
    private void TurnLeft(int pos) {
      resetEncoders();
      FrontLeft.setPower(power2);
      FrontLeft.setTargetPosition(pos);
      FrontRight.setPower(power2);
      FrontRight.setTargetPosition(pos);
      BackLeft.setPower(power2);
      BackLeft.setTargetPosition(pos);
      BackRight.setPower(power2);
      BackRight.setTargetPosition(pos);
      runToPos();
      while (FrontLeft.isBusy());
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
    
  private void initHardwareMap() {
    
    shoulder = hardwareMap.get(DcMotor.class, "shoulder");
    elbow = hardwareMap.get(DcMotor.class, "elbow");
    
    clawLeft = hardwareMap.get(Servo.class, "clawLeft");
    clawRight = hardwareMap.get(Servo.class, "clawRight");
    wrist = hardwareMap.get(Servo.class, "wrist");
    
    
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    
  }

  private void initHardwareConfig() {
    if (shoulder != null) {
      shoulder.setTargetPosition(0);
      shoulder.setPower(0.2);
      shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    if (elbow != null) {
      elbow.setTargetPosition(0);
      elbow.setPower(0.2);
      elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    if (BackLeft != null) {
      BackLeft.setTargetPosition(0);
      BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    if (clawLeft != null) {
      clawLeft.setDirection(Servo.Direction.FORWARD);
      clawLeft.setPosition(0);
    }

    if (clawRight != null) {
      clawRight.setDirection(Servo.Direction.REVERSE);
      clawRight.setPosition(0);
    }
    
    if (wrist != null) {
      wrist.scaleRange(0, 0.65);
      wrist.setDirection(Servo.Direction.FORWARD);
    }
    
    timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    // start position
    armState = ArmState.INIT;
    setArmPosition(0, 0, 0, 0, WRIST_UP, 0);
    clawPosition = CLAW_OPEN;
    
  }

  private void handleInput() {
    // debounce button press
    boolean currentState =
      gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b
      || gamepad1.right_bumper || gamepad1.left_bumper;
      
    if (lastState != currentState) {
      lastStateChangeTime = timer.milliseconds();
      lastState = currentState;
    }

    if (timer.milliseconds() - lastStateChangeTime > DEBOUNCE_MS) {
      if (currentState != lastSteadyState) {
        if (currentState) {
          onButtonPressed();
        }
        lastSteadyState = currentState;
      }
    }
  }
  
  private void onButtonPressed()
  {
    if (gamepad1.y) {
      if (armState != ArmState.PICK1) {
        armState = ArmState.PICK1;
        setArmPosition(5, 0, -55, 0, WRIST_UP, 0);
      } else {
        armState = ArmState.PICK2;
        setArmPosition(5, 0, -45, 0, WRIST_UP, 0);
      }
      hold = false;
    } else if (gamepad1.x) {
      if (armState != ArmState.SHORT_POLE1) {
        armState = ArmState.SHORT_POLE1;
        setArmPosition(45, 0, -35, 200, WRIST_UP, 0);
      } else {
        armState = ArmState.SHORT_POLE2;
        setArmPosition(45, 0, -55, 0, WRIST_UP, 0);
      }
      hold = true;
    } else if (gamepad1.a) {
    if (armState != ArmState.MIDDLE_POLE1) {
        armState = ArmState.MIDDLE_POLE1;
        setArmPosition(95, 0, -75, 200, WRIST_UP, 0);
      } else {
        armState = ArmState.MIDDLE_POLE2;
        setArmPosition(95, 0, -95, 0, WRIST_UP, 0);
      }
      hold = true;
    } else if (gamepad1.b) {
      if (armState != ArmState.TALL_POLE1) {
        armState = ArmState.TALL_POLE1;
        setArmPosition(180, 0, -75, 200, WRIST_DOWN, 500);
      } else {
        armState = ArmState.TALL_POLE2;
        setArmPosition(180, 0, -55, 0, WRIST_DOWN, 0 );
      }
      hold = false;
    } else if (gamepad1.right_bumper) {
      toggleClaw();
    } else if (gamepad1.left_bumper) {
      flipWrist();
    }
  }

  private void setArmPosition(
    int shoulderDegree, double sdelay,
    int elbowDegree, double edelay,
    double wristPos, double wdelay) {
    shoulderPosition = -shoulderDegree * 28;
    shoulderDelay = sdelay;
    elbowPosition = -elbowDegree * 20;
    elbowDelay = edelay;
    wristPosition = wristPos;
    wristDelay = wdelay;
    lastRequestTime = timer.milliseconds();
  }

  private void toggleClaw()
  {
    if (clawPosition == CLAW_OPEN) {
      clawPosition = CLAW_CLOSE;
    } else {
      clawPosition = CLAW_OPEN;
    }
  }
  
  private void flipWrist() {
    if (wristPosition == WRIST_UP) {
      wristPosition = WRIST_DOWN;
    } else {
      wristPosition = WRIST_UP;
    }
  }


  private void moveArm() {
    double delta = (timer.milliseconds() - lastRequestTime);
    
    if (delta > shoulderDelay) {
      moveMotor(shoulder, shoulderPosition);
    }
    if (delta > elbowDelay) {
      moveMotor(elbow, elbowPosition);
    }
    
    if (delta > wristDelay) {
      moveServo(wrist, wristPosition);
    }

    moveServo(clawLeft, clawPosition);
    moveServo(clawRight, clawPosition-0.15);
    

  }
  
  private void moveMotor(DcMotor motor, int pos)
  {
      if (motor != null){
        if (motor.getTargetPosition() != pos) {
          motor.setPower(SHOULDER_POWER);
          motor.setTargetPosition(pos);
        } else if (!motor.isBusy() && !hold) {
          motor.setPower(0);
        }
        else if (Math.abs(motor.getCurrentPosition() - pos) < SLOW_DOWN_THRESHOLD) {
          motor.setPower(SLOW_DOWN_POWER);
        }
      }
  }
  
  private void moveServo(Servo servo, double pos)
  {
      if (servo != null) {
        if (servo.getPosition() != pos) {
          servo.setPosition(pos);
        }
      }  
  }

  
  private void moveRobot() {
    double y_factor = 0.5;
    double x_factor = 0.5;
    double turn_factor = 0.5;
    
    if (FrontLeft != null) {
      double y = -gamepad2.left_stick_y * y_factor;
      double x = gamepad2.left_stick_x * x_factor;
      double rx = gamepad2.right_stick_x * turn_factor;

      FrontLeft.setPower(-(y + x + rx));
      BackLeft.setPower(-(y - x + rx));
      FrontRight.setPower(y - x - rx);
      BackRight.setPower(y + x - rx);
    }
  }

  private void showTelemetry() {

    telemetry.addData("arm state", armState);

    if (shoulder != null) {
      telemetry.addData("Shoulder targert", shoulder.getTargetPosition());
      telemetry.addData("Shoulder current", shoulder.getCurrentPosition());
      telemetry.addData("Shoulder power", shoulder.getPower());
    }

    if (elbow != null) {
      telemetry.addData("Elbow targert", elbow.getTargetPosition());
      telemetry.addData("Elbow current", elbow.getCurrentPosition());
      telemetry.addData("Elbow Power", elbow.getPower());
    }

    if (wrist != null) {
      telemetry.addData("Wrist targert", wristPosition);
      telemetry.addData("Wrist current", wrist.getPosition());
    }

    if (clawLeft != null && clawRight != null) {
      telemetry.addData("Claw targert", clawPosition);
      telemetry.addData("Claw left", clawLeft.getPosition());
      telemetry.addData("Claw right", clawRight.getPosition());
    }
    
    telemetry.addData("Status", "Running");
    telemetry.update();
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
       tfodParameters.minResultConfidence = 0.5f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
