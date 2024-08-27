`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module safe_box(
        input clk, reset_p,
        input [3:0] row,
        input echo,
        output trig,
        output [3:0] col,
        output [3:0] com,
        output [7:0] seg_7,
        output led_key_valid,
        output reg lock , ERROR,
        output [15:0] led_debug);
    
    
     //WIRE+REG
      reg [5:0] count;
      wire [15:0]timer_value;
      wire [3:0] key_value;
      wire key_valid;
      
      
     //CLOCK
     wire clk_usec, clk_msec, clk_sec;
     clock_div_100 clk_us(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
     clock_div_1000 clk_ms( .clk(clk) , .reset_p(reset_p) , .clk_source(clk_usec) , .clk_div_1000(clk_msec));
     clock_div_1000 clk_s( .clk(clk) , .reset_p(reset_p), .clk_source(clk_msec) , .clk_div_1000_nedge(clk_sec));  
     
     
     //EDGE_DETECTOR
     wire clk_sec_n, clk_div_26_nedge;
     edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p),.cp(clk_sec),  .n_edge(clk_sec_n)); 
     edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(key_valid),.p_edge(key_valid_p), .n_edge(clk_8msec_n));                                                                                               
     edge_detector_n clk_22(.clk(clk), .reset_p(reset_p),.cp(clk_div[22]), .n_edge(clk_div_26_nedge));      
    
      ////////////////////////////
      //키패드
      key_pad_cntr_FSM keypad(.clk(clk), .reset_p(reset_p), .row(row), .col(col), .key_value(key_value), .key_valid(key_valid));
     assign led_key_valid = key_valid;
     
     
     ///////////////////
     //서보모터 
     wire pwm;
     servo_motor safe_box(.clk(clk) , .reset_p(reset_p), .spi_keypad_data(),.ultra_data(), .servo_motor_pwm(pwm));
    
     /////////////////////
     //초음파
     wire [50:0] distance;
     ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo1(echo), .trig(trig),.distance(distance), .led_debug(led_debug));
    
    
     
    
     
     //MAIN_CODE
     reg [2:0] state; 
     always @(posedge clk or posedge reset_p) begin
         if (reset_p) begin
             lock <= 0;
             state <= 0;   // 초기 상태 설정
             ERROR <= 0;
         end else if (key_valid_p) begin
             case (state)
                 3'd0: begin
                     lock <= 0;
                     ERROR <= 0;
                     count = 60;
                     state <= 3'd1;
                 end
                 
                 3'd1: begin
                     if (key_value == 1)begin
                         state <= 3'd2;
                     end
                     else 
                         ERROR <= 1; // 잘못된 입력 시 에러 표시
                 end
     
                 3'd2: begin
                     if (key_value == 2)begin
                         state <= 3'd3;
                     end
                     else begin
                         state <= 3'd0;
                         ERROR <= 1;
                     end
                 end
     
                 3'd3: begin
                     if (key_value == 3)begin
                         state <= 3'd4;
                     end
                     else begin
                         state <= 3'd0;
                         ERROR <= 1;
                     end
                 end
     
                 3'd4: begin
                     if (key_value == 4) begin
                         lock <= 1;   // 올바른 시퀀스 완료 시 lock 설정
                         if(count == 0)begin
                             state = 3'd0   ;                 
                         end
                     end 
                     else begin
                         state <= 3'd0;
                         ERROR <= 1;
                     end
                 end
     
                 default: state <= 3'd0;
             endcase
         end
     end
     
     
     
     //하위 코드 정보
     //타이머 카운트 코드
     always @(posedge clk or posedge reset_p) begin
         if (reset_p) begin
             count <= 60; 
         end else if (lock) begin  // lock이 1일 때만 count 감소
             if (clk_sec_n) begin
                 if (count > 0)
                     count <= count - 1;
                 else
                     count <= 0;  // count가 음수가 되지 않도록
             end
         end
     end
     
     //서보모터코드
     reg [31:0] clk_div;    
     always @(posedge clk or posedge reset_p) begin
         if(reset_p)clk_div = 20;
         else clk_div = clk_div +1;
     end
    

    
     reg[6:0] duty;
     reg up_down;
     wire spi_keypad_data, ultra_data;
     always @(posedge clk or posedge reset_p) begin
         if(reset_p) begin
         duty =12;
         end
                        //조건문 편하게 바꾸기(바꿀꺼면 wire도 지우기
        else if(spi_keypad_data) begin 
             if(clk_div_26_nedge) begin
                if(duty<50)
                   duty =duty +1;
             end
        end
                       //편하게 바꾸기 (바꿀꺼면 wire도 지우기)
         else if(ultra_data) begin
             if(clk_div_26_nedge) begin
                if(duty >12)
                   duty =duty -1;
             end
         end   
      end
     
     pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
     pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(motor_pwm));
    
    
     bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
     fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                          .com(com), .seg_7(seg_7));

endmodule
