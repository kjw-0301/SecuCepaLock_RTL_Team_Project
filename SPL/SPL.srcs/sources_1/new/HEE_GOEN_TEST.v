`timescale 1ns / 1ps
//==========================================================
  module safe_box_top_hee_goen(
       input clk, reset_p,
       input btn_close, //�� ���½ÿ� ��ư ������ �� ���� off
       input [3:0] row,
       output [3:0] col,
       output scl,sda,
       output led_key_valid,
       output servo_motor_pwm,
       output reg open , close, ERROR,
       output reg led_test,
       output [3:0] com,
       output [7:0] seg_7,
       output reg [7:5] led,    //state 3'd5~7 Ȯ���� �����ϱ� �ӽ�led
                                            //���߿� reset_p���¿� state���� ���� �ϱ�
       output reg led_pw_push); //��й�ȣ�� ������ on�Ǵ� led
        
       reg stop_start;
       reg [5:0] count;
       reg [2:0] state; 
       reg count_on;    //count_on�� 
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
      //��������                                                               
      servo_motor smmt(.clk(clk), .reset_p(reset_p), .open(open),  .close(close), .btn_close(btn_close),
                                   .servo_motor_pwm(servo_motor_pwm));
                                                            
      //������
      ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo(echo),
                                                     .trig(trig),.distance(distance));
   
     
      //Ű�е� 
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
                    //3'd0�� ���ʿ��� ���� �ٽ� �����ʴ´�.(�� ����, �����ϰ� ó�� ��������)
                    //���� ó�� Ű�� �������� ��ŸƮ ��ư�� �ƴϸ� ������ �߸鼭 3'd0�� �ӹ���
                    //start��ư�� �������� 3'd1�� �̵�
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
      
                   3'd1: begin      //3'd1�� ���۵Ǹ� timer 10�� ����
                      ERROR <=0;
                      led_pw_push =1;
                      if (key_value == 1) begin
                          state <= 3'd2;
                      end 
                      else begin
                          ERROR <= 1; // �߸��� �Է� �� ���� ǥ��
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
                        open <= 1;   // �ùٸ� ������ �Ϸ� �� open ����
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
                  
                  //���� �������� 1~5�� ��ġ�� 6���� �´�.
                  //���⼱ 12���� ������ 3'd1(Ű�� ������ �۾����� ����)�� ���ư���
                  //�ٸ� ��ư�� ������3'd6�� �ӹ�����(�� ������)
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
                  
                  //��ŸƮ��ư(key_value == 8'd12)�� ������ ������ ��쿡 ���⿡ ���Եȴ�.
                  //���� ������ ������ üũ�Ǹ� ���������� ����.
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