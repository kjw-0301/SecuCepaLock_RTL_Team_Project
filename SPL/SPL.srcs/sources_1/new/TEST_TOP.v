`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////



//이거 hierachy에 이름검색하면 나옵니다
/*

//I2C
I2C_txtlcd_top(clk,reset_p,
    input [3:0] btn,
    output scl,sda,
    output [15:0] led_debug);

//1분타이머     >> 
//timer_1m(.clk(), .reset_p(), .out_data());

//서보모터   >> 타이머가 0보다크며, (led가 2개다 on일떄 or key_pad와 카드키를 모두 on했을떄 ) 
servo_motor(clk,reset_p,
    input [2:0] btn,
    output servo_motor_pwm);

//초음파  >> 거리가 30cm 이상 멀어지면 10초뒤 close
ultrasonic_sensor_cntr(clk,reset_p,echo1,trig,
        output reg [50:0] distance,
        output [7:0] led_debug);

//키패드
keypad_cntr_FSM (clk,reset_p,
    input [3:0] row,        //줄
    output reg [3:0] col,      //열
    output reg [3:0] key_value,
    output reg key_valid); 
    */