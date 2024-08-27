`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module safe_box_end(
    input clk,reset_p,
    input echo,
    output trig,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);


    //1��Ÿ�̸�    -> out_data�� spi����� on�Ǿ�����  Ÿ�̸Ӱ� 1�� �����ǰ���
    wire [5:0] count;
    wire spi_led_on;
    timer_1m timer(.clk(clk), .reset_p(reset_p), .count(count), .out_data(spi_led_on));

    //��������   >> Ÿ�̸Ӱ� 0����ũ��, (led�� 2���� on�ϋ� or key_pad�� ī��Ű�� ��� on������ ) 
    //��ġ ���� �ʿ���
    wire pwm;
    wire pwm_nstepmotor_pwm;
    servo_motor safe_box(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                         .ultra_data(), .servo_motor_pwm(pwm));
                                  
    servo_motor door(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                 .ultra_data(), .servo_motor_pwm(pwm));                               

    
    pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
    pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(motor_pwm));



//������  >> �Ÿ��� 30cm �̻� �־����� 10�ʵ� close
wire [50:0] distance;
ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo1(echo), .trig(trig),
                                                .distance(distance), .led_debug(led_debug));
   
   
/*
//I2C
I2C_txtlcd_top(clk,reset_p,
    input [3:0] btn,
    output scl,sda,
    output [15:0] led_debug);
   */
   
                                                
//Ű�е�
wire [3:0] row, col, key_value;
wire key_valid;
wire [3:0]key_number;

keypad_cntr_FSM  keypad(.clk(clk), .reset_p(reset_p), .row(row), .col(col),
                                           .key_value(key_value), .key_valid(key_valid), 
                                           .key_number(key_number)); 
/*
always @(posedge clk or posedge reset_p) begin
    if(reset_p) begin
    
    end
    else if(led 1�� on) begin
        if(key_number ==1) begin
            if(key_number ==2) begin
                if(key_number ==3) begin
                    if(key_number ==4) begin 
                        /* servo moter pwm ���ư���
                        else  begin
                            led1 off + ����
                        end
                        else begin
                            led1 off + ����
                         end
                         else begin
                            led1 off + ����
                         end
                     end
                   end  
                end
            end
          end
      end
  */

endmodule








