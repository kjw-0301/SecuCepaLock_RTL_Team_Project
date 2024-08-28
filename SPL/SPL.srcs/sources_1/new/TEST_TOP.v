`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
  module keypad_test_top(
        input clk, reset_p,
        input [3:0] row,
        output [3:0] col,
        output [3:0] com,
        output [7:0] seg_7,
        output led_key_valid,
        output reg open , ERROR);
 
     reg [5:0] count;
     wire [15:0]timer_value;
     wire [3:0] key_value;
     wire key_valid;
     
    wire clk_usec, clk_msec, clk_sec;
    clock_div_100 clk_us(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 clk_ms( .clk(clk) , .reset_p(reset_p) , 
                                      .clk_source(clk_usec) , .clk_div_1000(clk_msec));
    clock_div_1000 clk_s( .clk(clk) , .reset_p(reset_p) , 
                                    .clk_source(clk_msec) , .clk_div_1000_nedge(clk_sec));  
    wire clk_sec_n;
    edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p),   
                                          .cp(clk_sec),  .n_edge(clk_sec_n)); 
                                                                   
     
          /////////////////////
     //초음파
     wire [50:0] distance;
     ultrasonic_sensor_cntr ultra(.clk(clk), .reset_p(reset_p), .echo1(echo), .trig(trig),.distance(distance));
    
    ////////////////////////
    //서보모터
    wire  pwm ;
    reg [2:0] state; 
    //서보모터에 임의로 state를 넣었으니까 나중에 수정하기
    servo_motor lock(.clk(clk), .reset_p(reset_p), .spi_keypad_data(open == 1), .ultra_data(count == 0), .servo_motor_pwm(pwm));
     
     key_pad_cntr_FSM keypad(.clk(clk), .reset_p(reset_p),
    .row(row), .col(col), .key_value(key_value), .key_valid(key_valid));
    assign led_key_valid = key_valid;
    
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(key_valid), .p_edge(key_valid_p), .n_edge(clk_8msec_n));
    reg [15:0] key_count;
    
    //I2C 
//    I2C_txtLCD_top(clk, reset_p,btn,scl,sda,led);
    
    
    
   reg [2:0] state; 
    reg stop_start;
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            count <= 10; 
            stop_start <= 0;
        end else if (state && !open) begin  // open 1일 때만 count 감소
            if (clk_sec_n) begin
                if (count > 0 && !stop_start) begin
                    stop_start <= 0;
                    count <= count - 1;
                end else begin
                    stop_start <= 1;  // count가 0이 될 때 stop_start 설정
                    count <= 0;  // count가 0이 되면 10으로 초기화
                    
                end
            end
        end
    end

always @(negedge clk or posedge reset_p) begin
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
                if (key_value == 1) begin
                    state <= 3'd2;
                end else 
                    ERROR <= 1; // 잘못된 입력 시 에러 표시
            end

            3'd2: begin
                if (key_value == 2) begin
                    state <= 3'd3;
                end else begin
                    state <= 3'd0;
                    ERROR <= 1;
                end
            end

            3'd3: begin
                if (key_value == 3) begin
                    state <= 3'd4;
                end else begin
                    state <= 3'd0;
                    ERROR <= 1;
                end
            end
    
                3'd4: begin
                    if (key_value == 4) begin
                        open <= 1;   // 올바른 시퀀스 완료 시 open 설정
                        if(stop_start == 1 || distance > 30) begin
                            open <= 0;                        
                            state <= 3'd0;                 
                        end
                    end else begin
                        state <= 3'd0;
                        ERROR <= 1;
                    end
                end
    
                default: state <= 3'd0;
            endcase
        end
    end



    bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                      .com(com), .seg_7(seg_7));

    endmodule



