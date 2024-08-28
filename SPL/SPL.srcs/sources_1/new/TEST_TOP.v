`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module safe_box_end(
    input clk,reset_p,
    input echo,
    output trig,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);

    //서보모터   >> 타이머가 0보다크며, (led가 2개다 on일떄 or key_pad와 카드키를 모두 on했을떄 ) 
    //수치 조정 필요함
    wire pwm;
    wire pwm_nstepmotor_pwm;
    servo_motor safe_box(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                         .ultra_data(), .servo_motor_pwm(pwm));
                                  
    servo_motor door(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                 .ultra_data(), .servo_motor_pwm(pwm));                               

    




//초음파  >> 거리가 30cm 이상 멀어지면 10초뒤 close
wire [50:0] distance;
ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo1(echo), .trig(trig),
                                                .distance(distance), .led_debug(led_debug));


endmodule

