`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////



//�̰� hierachy�� �̸��˻��ϸ� ���ɴϴ�
/*

//I2C
I2C_txtlcd_top(clk,reset_p,
    input [3:0] btn,
    output scl,sda,
    output [15:0] led_debug);

//1��Ÿ�̸�     >> 
//timer_1m(.clk(), .reset_p(), .out_data());

//��������   >> Ÿ�̸Ӱ� 0����ũ��, (led�� 2���� on�ϋ� or key_pad�� ī��Ű�� ��� on������ ) 
servo_motor(clk,reset_p,
    input [2:0] btn,
    output servo_motor_pwm);

//������  >> �Ÿ��� 30cm �̻� �־����� 10�ʵ� close
ultrasonic_sensor_cntr(clk,reset_p,echo1,trig,
        output reg [50:0] distance,
        output [7:0] led_debug);

//Ű�е�
keypad_cntr_FSM (clk,reset_p,
    input [3:0] row,        //��
    output reg [3:0] col,      //��
    output reg [3:0] key_value,
    output reg key_valid); 
    */