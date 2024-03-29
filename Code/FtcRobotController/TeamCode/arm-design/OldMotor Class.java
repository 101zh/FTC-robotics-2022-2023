
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class MyMotor {

    private DcMotor motor = null;
    private float ratio_enum = 0;
    private float ratio_denum = 0;
    private float tgt_pos;
    private boolean running = false;
    private String type;
    
    public MyMotor(DcMotor m, float r_enum, float r_denum, String t) {
        motor = m;
        ratio_enum = r_enum;
        ratio_denum = r_denum;
        type = t;
    }
    
    public float get_current_postion() {
        return motor.getCurrentPosition();
    }
    public float get_current_degree() {
        float x = -(motor.getCurrentPosition()*ratio_denum)/ratio_enum;
        return x;
    }
    public void goto_pos_degree(float degree) {
                                    //^degree
        float math = (degree/ratio_denum)*ratio_enum;
        
        if (motor.getCurrentPosition() >= math){
            while (motor.getCurrentPosition() >= math) {
                motor.setPower(-1);
            }
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (motor.getCurrentPosition() <= math) {
            while (motor.getCurrentPosition() <= math) {
                
                motor.setPower(1);
            }
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    public void proceed() {
        if (motor.getCurrentPosition() <= tgt_pos+20 && motor.getCurrentPosition() >= tgt_pos-20) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            running = false;
        }
        
        if (running) {
            if (motor.getCurrentPosition() >= (tgt_pos)) {
                motor.setPower(-1);
            } else if (motor.getCurrentPosition() <= (tgt_pos)) {
                motor.setPower(1);
            }
        }
        return;
    }
    
    public void set_target_pos(float degree) {
        tgt_pos = (degree/ratio_denum)*ratio_enum;
        //tgt_pos/ratio_enum*ratio_denum
        running = true;
    }
    
    public float get_target_pos() {
        return tgt_pos;
    }
    
    public boolean is_running() {
        return running;
    }
}
