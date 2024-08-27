`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module safe_box_end(
    input clk,reset_p,
    input echo,
    output trig,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);


    //1분타이머    -> out_data에 spi통신이 on되었을떄  타이머가 1분 설정되게함
    wire [5:0] count;
    wire spi_led_on;
    timer_1m timer(.clk(clk), .reset_p(reset_p), .count(count), .out_data(spi_led_on));

    //서보모터   >> 타이머가 0보다크며, (led가 2개다 on일떄 or key_pad와 카드키를 모두 on했을떄 ) 
    //수치 조정 필요함
    wire pwm;
    wire pwm_nstepmotor_pwm;
    servo_motor safe_box(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                         .ultra_data(), .servo_motor_pwm(pwm));
                                  
    servo_motor door(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),
                                 .ultra_data(), .servo_motor_pwm(pwm));                               

    
    pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
    pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(motor_pwm));



//초음파  >> 거리가 30cm 이상 멀어지면 10초뒤 close
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
   
                                                
//키패드
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
    else if(led 1번 on) begin
        if(key_number ==1) begin
            if(key_number ==2) begin
                if(key_number ==3) begin
                    if(key_number ==4) begin 
                        /* servo moter pwm 돌아가기
                        else  begin
                            led1 off + 리셋
                        end
                        else begin
                            led1 off + 리셋
                         end
                         else begin
                            led1 off + 리셋
                         end
                     end
                   end  
                end
            end
          end
      end
  */

endmodule








