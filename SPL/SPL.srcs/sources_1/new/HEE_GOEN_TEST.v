`timescale 1ns / 1ps
//==========================================================
  module safe_box_top_hee_goen(
       input clk, reset_p,
       input btn_close, //문 오픈시에 버튼 누르면 문 강제 off
       input [3:0] row,
       output [3:0] col,
       output scl,sda,
       output led_key_valid,
       output servo_motor_pwm,
       output reg open , close, ERROR,
       output reg led_test,
       output [3:0] com,
       output [7:0] seg_7,
       output reg [7:5] led,    //state 3'd5~7 확인이 어려우니까 임시led
                                            //나중에 reset_p상태와 state에서 제거 하기
       output reg led_pw_push); //비밀번호를 누를때 on되는 led
        
       reg stop_start;
       reg [5:0] count;
       reg [2:0] state; 
       reg count_on;    //count_on과 
       wire btn_stable;
       wire [3:0] key_value;
       wire key_valid;
       wire clk_usec, clk_msec, clk_sec, clk_sec_n;
       wire [50:0] distance;
       wire [15:0]timer_value;
     
       //CLOCK
      clock_div_100 clk_us(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
      clock_div_1000 clk_ms(.clk(clk) , .reset_p(reset_p) ,.clk_source(clk_usec) ,
                                             .clk_div_1000(clk_msec));
      clock_div_1000 clk_s(.clk(clk) , .reset_p(reset_p) ,  .clk_source(clk_msec) , 
                                         .clk_div_1000_nedge(clk_sec));  
     
      //EDGE_DETECTOR
      edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p), .cp(clk_sec), .n_edge(clk_sec_n)); 
      edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(key_valid),
                                       .p_edge(key_valid_p), .n_edge(clk_8msec_n));
       
      //INST
      //서보모터                                                               
      servo_motor smmt(.clk(clk), .reset_p(reset_p), .open(open),  .close(close), .btn_close(btn_close),
                                   .servo_motor_pwm(servo_motor_pwm));
                                                            
      //초음파
      ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo(echo),
                                                     .trig(trig),.distance(distance));
   
     
      //키패드 
      key_pad_cntr_FSM keypad(.clk(clk), .reset_p(reset_p), .row(row), 
                                                  .col(col), .key_value(key_value), .key_valid(key_valid));
     
     //I2C 
     I2C_txtLCD_top lcd(.clk(clk),.reset_p(reset_p),.scl(scl),.sda(sda),
                                     .key_valid(key_valid_p),.open(open),.error(ERROR));
    
      
       //MAIN_CODE
      always @(negedge clk or posedge reset_p) begin
          if (reset_p) begin
              count <= 0; 
              led_test = 0;
          end 
          
          else if((count_on ==1) && (key_value == 8'd12)) begin
                count =10;
          end
              
          else if (clk_sec_n) begin
            if (count > 0) begin 
                count <= count - 1;
            end 
          end
              
          else if (open == 1)begin
               count <= 0;
          end
      end
    
     
      always @(posedge clk or posedge reset_p) begin
          if (reset_p) begin
              open <= 0;
              close <= 1;
              ERROR <= 0;
              state <= 0;
              count_on =1;
              led_pw_push <= 0;
              //
              led[5] =0;
              led[6] =0;
              led[7] =0;
          end 
          
          
       else if(key_valid_p) begin
            case(state)
                    //3'd0은 최초에만 오며 다시 오지않는다.(즉 리셋, 세팅하고 처음 눌렀을때)
                    //가장 처음 키가 눌렸을떄 스타트 버튼이 아니면 에러가 뜨면서 3'd0에 머물고
                    //start버튼이 눌렸으면 3'd1로 이동
                   3'd0 : begin
                     ERROR =0;
                     count_on =1;
                     if(key_value == 8'd12) begin
                         led_pw_push =1;
                         ERROR <=0;
                         state <= 3'd1;
                         count_on =0;
                     end
                     else begin
                         ERROR <=1;
                         state <= 3'd0;
                     end
                  end
      
                   3'd1: begin      //3'd1이 시작되면 timer 10초 시작
                      ERROR <=0;
                      led_pw_push =1;
                      if (key_value == 1) begin
                          state <= 3'd2;
                      end 
                      else begin
                          ERROR <= 1; // 잘못된 입력 시 에러 표시
                          state <= 3'd7;
                      end
                  end
      
                  3'd2: begin
                    led_pw_push =1;
                    if (key_value == 2) begin
                        state <= 3'd3;
                    end 
                    else begin
                        state <= 3'd7;
                        ERROR <= 1;
                    end
                  end
      
                  3'd3: begin
                     led_pw_push =1;
                     if (key_value == 3) begin
                         state <= 3'd4;
                     end 
                     else begin
                         state <= 3'd7;
                         ERROR <= 1;
                     end
                  end
          
                  3'd4: begin
                     led_pw_push =1;
                     if (key_value ==4 && close) begin
                        close =0;
                        open <= 1;   // 올바른 시퀀스 완료 시 open 설정
                        led_pw_push =0;
                        state <= 3'd5;
                     end 
                     else begin
                        ERROR <= 1;
                        state <= 3'd7;
                     end
                  end
                  
                  3'd5: begin  
                    led[5] =1;
                    led_pw_push =0;
                      if (key_value == 8'd15 && open) begin 
                          open = 0;
                          close = 1;
                          led_test =1;
                          count_on =1;
                          state <= 3'd6;
                      end 
                      else begin
                          state <= 3'd5;
                      end
                  end
                  
                  //정상 동작으로 1~5를 거치면 6으로 온다.
                  //여기선 12번을 누르면 3'd1(키를 누르는 작업부터 시작)로 돌아가며
                  //다른 버튼을 누르면3'd6에 머무른다(즉 대기상태)
                  3'd6: begin
                    led[5] = 0;
                    led[6] = 1;
                    open <= 0;
                    close <= 1;
                    ERROR <= 0;
                      if(key_value == 8'd12) begin
                          count_on =0;
                          led[6] =0;
                          state <= 3'd1;
                      end
                      else if(key_value != 8'd12) begin
                          state <= 3'd6;
                      end
                  end   
                  
                  //스타트버튼(key_value == 8'd12)을 누르고 실패한 경우에 여기에 오게된다.
                  //문이 닫히고 에러는 체크되며 빠져나갈수 없다.
                  3'd7: begin
                    led[6] =0;
                    led[7] =1;
                    open <= 0;
                    close <= 1;
                    ERROR <= 1;
                  end
                   
                endcase
            end
      end    

     assign led_key_valid = key_valid;
     
      bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
      fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),.com(com), .seg_7(seg_7));
                                           
endmodule