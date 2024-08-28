`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
  module keypad_test_top(
        input clk, reset_p,
        input [3:0] row,
        output [3:0] col,
        output [3:0] com,
        output [7:0] seg_7,
        output led_key_valid,
        output reg lock , ERROR);
 
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
                                                                   
     
     key_pad_cntr_FSM keypad(.clk(clk), .reset_p(reset_p),
    .row(row), .col(col), .key_value(key_value), .key_valid(key_valid));
    assign led_key_valid = key_valid;
    
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(key_valid), .p_edge(key_valid_p), .n_edge(clk_8msec_n));
    reg [15:0] key_count;
    
   reg [2:0] state; 
    reg stop_start;
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            count <= 10; 
            stop_start <= 0;
        end else if (lock) begin  // lock이 1일 때만 count 감소
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
        lock <= 0;
        state <= 0;   // 초기 상태 설정
        ERROR <= 0;
        stop_start <= 0;
    end else if (key_valid_p) begin
        case (state)
            3'd0: begin
                lock <= 0;
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
                        lock <= 1;   // 올바른 시퀀스 완료 시 lock 설정
                        if(stop_start == 1) begin
                            lock <= 0;                        
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

//////////////LCD//////////////
module I2C_txtLCD_top(
    input clk, reset_p,
    input[3:0] btn,
    output scl,sda,
    output[15:0]led);
    
    parameter IDLE = 6'b00_0001;
    parameter INIT = 6'b00_0010;
    parameter SEND_DATA = 6'b00_0100;
    parameter SEND_COMMAND_LINE_D = 6'b00_1000;
    parameter SEND_COMMAND_LINE_U = 6'b01_0000;
    parameter SEND_COMMAND_STRING = 6'b10_0000;
    
    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p),.clk_div_100_nedge(clk_microsec));
    
    reg[21:0] count_microsec;
    reg count_microsec_enable;
    always@(negedge clk or posedge reset_p)begin
        if(reset_p) count_microsec = 0;
        else if(clk_microsec && count_microsec_enable) count_microsec = count_microsec + 1;
        else if(!count_microsec_enable)count_microsec = 0;
    end
    
    wire[3:0]btn_pedge;
    button_Controller btn0(.clk(clk), .reset_p(reset_p),.btn(btn[0]), .btn_posedge(btn_pedge[0]));
    button_Controller btn1(.clk(clk), .reset_p(reset_p),.btn(btn[1]), .btn_posedge(btn_pedge[1]));
    button_Controller btn2(.clk(clk), .reset_p(reset_p),.btn(btn[2]), .btn_posedge(btn_pedge[2]));
    button_Controller btn3(.clk(clk), .reset_p(reset_p),.btn(btn[3]), .btn_posedge(btn_pedge[3]));
    
    reg[7:0] send_buffer;
    reg rs,send;
    
    wire busy; 
    I2C_LCD_send_byte lcd(.clk(clk),.reset_p(reset_p), .addr(7'h27), .send_buffer(send_buffer),.rs(rs),.send(send),.scl(scl),.sda(sda), .busy(busy),.led(led));
    
    
    reg[5:0] state, next_state;
    always@(negedge clk or posedge reset_p)begin
        if(reset_p) state = IDLE; 
        else state = next_state;
    end
    
    reg init_flag;
    reg[5:0] data_count;
    reg[8*14-1:0] init_word;
    reg[3:0] cnt_string;
    always@(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state = IDLE;
            init_flag = 0;
            data_count = 0;
            count_microsec_enable = 0;
            init_word = "Enter Passward";
            cnt_string = 14;
        end
        else begin
            case(state)
                IDLE:begin
                    if(init_flag)begin
                        if(!busy)begin
                            if(btn_pedge[0]) next_state = SEND_DATA;
                            if(btn_pedge[1]) next_state = SEND_COMMAND_LINE_D;
                            if(btn_pedge[2]) next_state = SEND_COMMAND_LINE_U;
                            if(btn_pedge[3]) next_state = SEND_COMMAND_STRING;
                        end
                    end
                    else begin
                        if(count_microsec <= 22'd80_000)begin
                            count_microsec_enable = 1;
                        end
                        else begin
                            next_state = INIT;    
                            count_microsec_enable = 0;
                        end
                    end
                end
                INIT:begin
                     if(busy)begin
                        send = 0;
                        if(data_count > 21)begin
                            next_state = IDLE;
                            init_flag = 1;        
                            data_count = 0;    
                            rs = 0;     
                        end
                    end
                    else if(!send) begin //s
                        case(data_count)
                            0: send_buffer = 8'h33;
                            1: send_buffer = 8'h32;
                            2: send_buffer = 8'h28;
                            3: send_buffer = 8'h0F;
                            4: send_buffer = 8'h01;
                            5: send_buffer = 8'h06;
                            6: send_buffer =  init_word[111:104];
                            7: send_buffer =  init_word[103:96]; 
                            8: send_buffer =  init_word[95:88];  
                            9: send_buffer =  init_word[87:80];  
                            10: send_buffer = init_word[79:72];  
                            11: send_buffer = init_word[71:64];  
                            12: send_buffer = init_word[63:56];  
                            13: send_buffer = init_word[55:48];  
                            14: send_buffer = init_word[47:40];  
                            15: send_buffer = init_word[39:32];  
                            16: send_buffer = init_word[31:24];  
                            17: send_buffer = init_word[23:16];  
                            18: send_buffer = init_word[15:8];   
                            19: send_buffer = init_word[7:0];                             
                            20: send_buffer = 8'h06;                             
                            21: send_buffer = 8'hC0;                             
                        endcase
                        if(data_count <= 5) rs=0;
                        else if(data_count > 5 && data_count < 20)rs = 1;
                        else if(data_count > 19)rs = 0;
                        send = 1;
                        data_count = data_count + 1;
                    end
                end
                    SEND_DATA:begin
                    if(busy)begin
                        next_state = IDLE;
                        send = 0;
                        if(data_count >= 9) data_count = 0;
                        else data_count = data_count + 1;
                    end
                    else begin
                        send_buffer = "0" +data_count;
                        rs = 1;
                        send = 1;
                    end
                end
                SEND_COMMAND_LINE_D:begin
                    if(busy)begin
                        next_state = IDLE;
                        send = 0;
                        if(data_count >= 9) data_count = 0;
                        else data_count = data_count + 1;
                    end
                    else begin
                        send_buffer = 8'hC0;
                        rs = 0;
                        send = 1;
                    end 
                end
                SEND_COMMAND_LINE_U:begin
                    if(busy)begin
                        next_state = IDLE;
                        send = 0;
                        if(data_count >= 9) data_count = 0;
                        else data_count = data_count + 1;
                    end
                    else begin
                        send_buffer = 8'h80;
                        rs = 0;
                        send = 1;
                    end 
                end
                SEND_COMMAND_STRING:begin
                    if(busy)begin
                        send = 0;
                        if(cnt_string < 1)begin
                            next_state = IDLE;
                            cnt_string = 14;    
                        end
                    end
                    else if(!send) begin //s
                        case(cnt_string)
                            14: send_buffer = init_word[111:104];
                            13: send_buffer = init_word[103:96]; 
                            12: send_buffer = init_word[95:88];  
                            11: send_buffer = init_word[87:80];  
                            10: send_buffer = init_word[79:72];  
                            9: send_buffer =  init_word[71:64];  
                            8: send_buffer =  init_word[63:56];  
                            7: send_buffer =  init_word[55:48];  
                            6: send_buffer =  init_word[47:40];  
                            5: send_buffer =  init_word[39:32];  
                            4: send_buffer =  init_word[31:24];  
                            3: send_buffer =  init_word[23:16];  
                            2: send_buffer =  init_word[15:8];   
                            1: send_buffer =  init_word[7:0];    
                        endcase
                        rs = 1;
                        send = 1;
                        cnt_string = cnt_string - 1;
                    end
                end
            endcase
        end
    
    end
    
    
    
endmodule


