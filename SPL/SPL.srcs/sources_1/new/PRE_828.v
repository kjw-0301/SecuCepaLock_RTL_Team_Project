`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
  module safe_box_top_PRE_828(
       input clk, reset_p,
       input close,
       input [3:0] row,
       output [3:0] col,
       output scl,sda,
       output led_key_valid,
       output servo_motor_pwm,
       output reg open , ERROR,
       output reg led_test,
       output [3:0] com,
       output [7:0] seg_7);
        
     reg stop_start;
     reg [5:0] count;
     reg [2:0] state; 
     wire [3:0] key_value;
     wire key_valid;
     wire clk_usec, clk_msec, clk_sec, clk_sec_n;
     wire [50:0] distance;
     wire [15:0]timer_value;
     
     //CLOCK
    clock_div_100 clk_us(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 clk_ms( .clk(clk) , .reset_p(reset_p) , 
                                      .clk_source(clk_usec) , .clk_div_1000(clk_msec));
    clock_div_1000 clk_s( .clk(clk) , .reset_p(reset_p) , 
                                    .clk_source(clk_msec) , .clk_div_1000_nedge(clk_sec));  
   
    //EDGE_DETECTOR
    edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p), .cp(clk_sec), .n_edge(clk_sec_n)); 
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(key_valid),
                                     .p_edge(key_valid_p), .n_edge(clk_8msec_n));
     
    //INST
    //서보모터                                                               
    servo_motor smmt(.clk(clk), .reset_p(reset_p), .open(open),  .close(close), 
                                   .out_data(), .servo_motor_pwm(servo_motor_pwm));
                                                            
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
            count <= 10; 
            stop_start <= 0;
            led_test = 0;
        end else if (state && !open) begin  // open 1일 때만 count 감소
            if (clk_sec_n) begin
                if (count > 0 && !stop_start) begin
                    stop_start <= 0;
                    count <= count - 1;
                end
                else if (close)begin
                    count = 10;
                end
                 else begin
                    stop_start <= 1;  // count가 0이 될 때 stop_start 설정
                    count <= 0;  // count가 0이 되면 10으로 초기화
                end
            end
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            open <= 0;
            state <= 0;   // 초기 상태 설정
            ERROR <= 0;
            stop_start <= 0;
        end else if (key_valid_p) begin
            case (state)
                3'd0: begin
                    open <= 0;
                    ERROR <= 0;
                    state <= 3'd1;
                end
                
                3'd1: begin
                    if (key_value == 8'd12) begin
                        state <= 3'd2;
                    end else begin
                        ERROR <= 1; // 잘못된 입력 시 에러 표시
                    end
                end
                
                3'd2: begin
                    if (key_value == 1) begin
                        state <= 3'd3;
                    end else begin
                        ERROR <= 1; // 잘못된 입력 시 에러 표시
                    end
                end
    
                3'd3: begin
                    if (key_value == 2) begin
                        state <= 3'd4;
                    end else begin
                        state <= 3'd0;
                        ERROR <= 1;
                    end
                end
    
                3'd4: begin
                    if (key_value == 3) begin
                        state <= 3'd5;
                    end else begin
                        state <= 3'd0;
                        ERROR <= 1;
                    end
                end
        
                3'd5: begin
                    if (key_value == 4 && !close) begin
                        open <= 1;   // 올바른 시퀀스 완료 시 open 설정
                        state <= 3'd6;
                    end else begin
                        state <= 3'd0;
                        ERROR <= 1;
                    end
    //                else if (open && close) begin
    //                    open <= 0;
    //                    state <= 3'd0;
    //                end 
                end
                
                3'd6: begin
                    if (key_value == 8'd15 && !close) begin
                        open = 0;
                        led_test =1;
                        state <= 3'd0;
                    end else begin
                        state <= 3'd0;
                    end
                end
                
    
                default: begin
                    state <= 3'd0;
                    ERROR <= 1;
                end
            endcase
        end
    end

    assign led_key_valid = key_valid;

    bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                      .com(com), .seg_7(seg_7));

  endmodule