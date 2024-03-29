package org.firstinspires.ftc.teamcode2.common.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "PowerPlayOpMode")
public class PowerPlayOpMode extends LinearOpMode {

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

  @Override
  public void runOpMode() {
    initHardwareMap();

    waitForStart();

    initHardwareConfig();

    while (opModeIsActive()) {
      handleInput();
      moveArm();
      moveRobot();
      showTelemetry();
    }
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
          shoulder.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);
          if (gamepad2.right_bumper) {
            while (gamepad2.right_bumper) {
              elbow.setPower(1);
              moveRobot();
              shoulder.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            }
            elbow.setPower(0);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          }
          if (gamepad2.left_bumper) {
            while (gamepad2.left_bumper) {
              elbow.setPower(-1);
              moveRobot();
              shoulder.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            }
            elbow.setPower(0);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          }
          //raw_elbow.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x)/2);
        
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
        setArmPosition(180, 0, -80, 200, WRIST_DOWN, 500);
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
    double turn_factor = 0.4;
    
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

}
