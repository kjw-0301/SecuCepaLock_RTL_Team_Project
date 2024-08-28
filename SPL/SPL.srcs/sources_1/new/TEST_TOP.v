`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module safe_box_end(
    input clk,reset_p,
    input echo,
    output trig,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);

    //��������   >> Ÿ�̸Ӱ� 0����ũ��, (led�� 2���� on�ϋ� or key_pad�� ī��Ű�� ��� on������ ) 
    //��ġ ���� �ʿ���
    wire pwm;
    wire pwm_nstepmotor_pwm;
    servo_motor safe_box(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                         .ultra_data(), .servo_motor_pwm(pwm));
                                  
    servo_motor door(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                 .ultra_data(), .servo_motor_pwm(pwm));                               

    




//������  >> �Ÿ��� 30cm �̻� �־����� 10�ʵ� close
wire [50:0] distance;
ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo1(echo), .trig(trig),
                                                .distance(distance), .led_debug(led_debug));


endmodule

