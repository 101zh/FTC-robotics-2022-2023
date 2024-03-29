package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "POCtest")
public class POCtest2 extends LinearOpMode {

  private DcMotor raw_shoulder;
  private DcMotor raw_elbow;
  private DcMotor BackRight;
  private DcMotor BackLeft;
  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private Servo raw_claw;
  private Servo raw_wrist;

  private MyMotor shoulder;
  private MyMotor elbow;
  
  String armStage;
  boolean first;
  int s = 0; //offset 26
  int e = -7; //offset -8
  
  private enum ArmStatus {
    ARM_STOPPED,
    ARM_UP_STEP_1,
    ARM_UP_STEP_2,
    ARM_UP_STEP_3,
    ARM_UP_STEP_4,
    ARM_UP_STEP_5,
    ARM_UP_STEP_6,
    ARM_DOWN_STEP_1,
    ARM_DOWN_STEP_2,
    DROP_CONE_STEP_1,
    DROP_CONE_STEP_2;
  };
  private ArmStatus arm_state = ArmStatus.ARM_STOPPED;
  
  private float set_servo_pos(float degree) {
        float x = (degree/300);
        return x;
  }
  private void arm_up() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_2;
    }
  }
  private void arm_up_back() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_5;
    }
  }
  private void drop_cone() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.DROP_CONE_STEP_1;
    }
  }
  private void arm_down_bottom() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_4;
    }
  }
  private void arm_up_top() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_2;
    }
  }
  private void handle_arm_state() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      switch(arm_state) {
        case ARM_STOPPED:
          // do nothing
          break;
        case ARM_UP_STEP_1:
          elbow.set_target_pos(20+e);
          first = false;
          arm_state = ArmStatus.ARM_UP_STEP_2;
          break;
        case ARM_UP_STEP_2: //drop fwrd
          shoulder.set_target_pos(-249+s);
          elbow.set_target_pos(177+e);
          armStage = "frwd";
          arm_state = ArmStatus.ARM_UP_STEP_3;
          break;
        case ARM_UP_STEP_3:
          break;
        case ARM_UP_STEP_4: //pickup
          if (armStage == "bkwd") {
            elbow.set_target_pos(95+e);
          }
          arm_state = ArmStatus.ARM_UP_STEP_6;
          break;
          /*
          s: 7267
          e: -1351
          
          s: 6870
          e: -1226
          */ 
        case ARM_UP_STEP_5: //drop bkwd
          shoulder.set_target_pos(-213+s);
          elbow.set_target_pos(48+e);
          armStage = "bkwd";
          arm_state = ArmStatus.ARM_STOPPED;
          break;
        case ARM_UP_STEP_6:
          raw_claw.setPosition(.25);
          shoulder.set_target_pos(-85+s);
          elbow.set_target_pos(100+e);
          armStage = "";
          arm_state = ArmStatus.ARM_STOPPED;
          break;
        case DROP_CONE_STEP_1:
          if (armStage == "frwd") {
            elbow.set_target_pos(200+e);
            shoulder.set_target_pos(-260+s);
          }
          if (armStage == "bkwd") {
            elbow.set_target_pos(60+e);
            shoulder.set_target_pos(-208+s);
          }
          arm_state = ArmStatus.DROP_CONE_STEP_2;
          break;
        case DROP_CONE_STEP_2:
          raw_claw.setPosition(.25);
          sleep(1000);
          arm_state = ArmStatus.ARM_STOPPED;
          break;
      }        
    }
  }
  // looking from right side, ccw is positive
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
    double Speed;
    double Turn;
    double Strafe;
    double FrontLeft2;
    double FrontRight2;
    double BackLeft2;
    double BackRight2;
    
  @Override
  public void runOpMode() {
    raw_shoulder = hardwareMap.get(DcMotor.class, "shoulder");
    raw_elbow = hardwareMap.get(DcMotor.class, "elbow");
    raw_claw = hardwareMap.get(Servo.class, "claw");
    raw_wrist = hardwareMap.get(Servo.class, "wrist");
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
    
    raw_claw.setPosition(0);
    
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      //raw_wrist.setPosition(.1);
      while (opModeIsActive()) {
        
        float pos = set_servo_pos(elbow.get_current_degree()+shoulder.get_current_degree()+65);
        raw_wrist.setPosition(pos);
        
        telemetry.addData("testshoulder+", shoulder.get_current_postion());
        telemetry.addData("testshoulder-", shoulder.get_target_pos());
        telemetry.addData("testelbow+", elbow.get_current_postion());
        telemetry.addData("testelbow-", elbow.get_target_pos());
        telemetry.addData("arm pos", arm_state);
        telemetry.update();
        
        shoulder.proceed();
        elbow.proceed();
        handle_arm_state();
        
        if (!shoulder.is_running() && !elbow.is_running()) {
          raw_shoulder.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
          if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) {
              raw_elbow.setPower(1);
              shoulder.proceed();
              elbow.proceed();
              handle_arm_state();
              MecanumWheelDrive();
              raw_shoulder.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            }
            raw_elbow.setPower(0);
            raw_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          }
          if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) {
              raw_elbow.setPower(-1);
              shoulder.proceed();
              elbow.proceed();
              handle_arm_state();
              MecanumWheelDrive();
              raw_shoulder.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            }
            raw_elbow.setPower(0);
            raw_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          }
          //raw_elbow.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x)/2);
        }
        if (gamepad1.x) {
          if (first = true) {
            arm_up();
          }
          else {
            arm_up_top();
          }
        }
        if (gamepad1.y) {
          arm_down_bottom();
        }
        if (gamepad1.b) {
          arm_up_back();
        }
        if (gamepad1.dpad_down) {
          drop_cone();
        }
        
        clawLoop += 1;
        wristLoop += 1;
        
        
        if (gamepad1.a && clawLoop >= 25) {
          if (clawMode == 0) {
            raw_claw.setPosition(.25);
            clawMode = 1;
            clawLoop = 1;
          } else if (clawMode == 1) {
            raw_claw.setPosition(0);
            clawMode = 0;
            clawLoop = 1;
          }
        } 
        MecanumWheelDrive();
      }
    }
  }
  private void MecanumWheelDrive() {
        
        Speed = -gamepad1.left_stick_y * 0.5;
        Turn = gamepad1.right_stick_x * 0.45;
        Strafe = -gamepad1.left_stick_x * 0.5;
        FrontLeft2 = (Speed - Turn) + Strafe;
        FrontRight2 = (Speed + Turn) - (-Strafe);
        BackLeft2 = (Speed - Turn) - Strafe;
        BackRight2 = Speed + Turn + (-Strafe);
        
        FrontLeft.setPower(-FrontLeft2);
        FrontRight.setPower(FrontRight2);
        BackLeft.setPower(-BackLeft2);
        BackRight.setPower(BackRight2);
    }
}
