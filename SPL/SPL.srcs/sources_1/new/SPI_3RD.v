`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////


//========================================
`timescale 1ns / 1ps

module RFID_RC522_SPI(
    input clk,               // 시스템 클럭
    input reset_n,           // 리셋 신호 (active low)
    output reg sck,          // SCK (Serial Clock)
    output reg mosi,         // MOSI (Master Out Slave In)
    input miso,              // MISO (Master In Slave Out)
    output reg nss,          // NSS (Slave Select, SDA)
    input irq,               // IRQ (Interrupt Request)
    output reg rst,         // RST (Reset)    ,
    output reg [15:0]led_debug 
);
    reg [7:0] received_data; // 수신한 데이터 저장

    wire clk_div_10_nedge;  //us초
    clock_div_10(.clk(clk_start), .reset_p(reset_p),.clk_div_10_nedge(sck)); 
    
    
    //SCK
    wire sck_p_edge, sck_n_edge;
    edge_detector_n(.clk(clk_start), .reset_p(reset_p) ,.cp(sck), .n_edge(sck_n_edge), .p_edge(sck_p_edge));    
    // 상태 머신 정의
    reg [2:0] state;
    reg [7:0] addr_to_send;
    reg [7:0] data_to_send;
    reg [2:0] bit_counter;

    // 상태 정의
    parameter IDLE = 3'b000;
    parameter SEND_ADDR = 3'b001;
    parameter SEND_DATA = 3'b010;
    parameter READ_DATA = 3'b011;
    parameter DONE = 3'b100;
    reg [2:0] next_state;
    
    always @(negedge clk or posedge reset_n) begin
        if(reset_n) state =IDLE;
        else state =next_state;
    end
    


    always @(posedge clk or negedge reset_n) begin
        if (reset_n) begin
            next_state <= IDLE;
            nss <= 1;           // 슬레이브 선택 해제
            mosi <= 1'b0;
            bit_counter <= 3'd7;
            received_data <= 8'b0;
        end else begin
            case (state)
                IDLE: begin
                    led_debug[0] =1;
                    nss <= 0; // 슬레이브 선택 (NSS 낮춤)
                    addr_to_send <= 8'h01; 
                    data_to_send <= 8'h0F; 
                    next_state <= SEND_ADDR;
                end

                SEND_ADDR: begin
                    led_debug[1] =1;
                    mosi <= addr_to_send[bit_counter];
                    if (bit_counter == 0) begin
                        bit_counter <= 3'd7;
                        next_state <= SEND_DATA;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end

                SEND_DATA: begin
                    led_debug[2] =1;
                    mosi <= data_to_send[bit_counter];
                    if (bit_counter == 0) begin
                        bit_counter <= 3'd7;
                        next_state <= READ_DATA;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end

                READ_DATA: begin
                    led_debug[3] =1;
                    received_data[bit_counter] <= miso;
                    if (bit_counter == 0) begin
                        next_state <= DONE;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end

                DONE: begin
                    led_debug[4] =1;
                    nss <= 1; // 슬레이브 선택 해제 (NSS 높임)
                    next_state <= IDLE;
                    
                end
            endcase
        end
    end
endmodule



//===============================
module SPI_Master(
    input clk,
    input MISO, // MASTER IN SLAVE OUT,
    input sw,
    input reset_p,
    output reg MOSI, // MASTER OUT SLAVE IN.
    output SCLK, // SPI Clock.
    output CS, // CHIP Select
    output [3:0] com,
    output [7:0] seg_7,
    output reg [4:0] led_debug
    /*output reg [7:0] Received_Data */
    );
reg [7:0] Received_Data;
    

    
    
    
    parameter [7:0] INIT_CODE = 8'b0000_1001;
    parameter [7:0] INIT_ADDRESS = 8'b0000_1011;

    localparam IDLE = 0 , INSTRUCTION = 1, ADDRESS = 2, DATA = 3, STOP = 4;

    reg cs_value = 1;
    reg [7:0] data_write_buffer_address = 8'b0000_1011;
    reg [7:0] data_write_buffer_instruction = 8'b0000_1001;
    reg [7:0] data_read_buffer;
    reg [3:0] data_counter = 0;
    reg [2:0] state;
    
    assign CS = cs_value;
    Clock_Divider divider (.clk(clk),.reset(reset),.SCLK(SCLK));

    always @ (negedge SCLK or posedge reset) begin
        if(reset) begin
            state <= IDLE;
            data_counter <= 0;
             led_debug = 0;
             Received_Data =0;
        end
        else begin
            case(state)
                IDLE : begin                
                    if(sw) begin
                       cs_value <= 0;
                       state <= INSTRUCTION;
                       data_counter <= 0;
                     end
                     else begin
                       cs_value <= 1;
                       state <= IDLE;                              
                     end                  
                end
                INSTRUCTION : begin
                    if(data_counter == 7) begin
                        data_counter <= 0;
                        state <= ADDRESS;
                    end
                    else begin
                        data_counter <= data_counter + 1;
                        state <= INSTRUCTION;
                    end
                end
                ADDRESS : begin
                if(data_counter == 7) begin
                        data_counter <= 0;
                        state <= DATA;
                    end
                    else begin
                        data_counter <= data_counter + 1;
                        state <= ADDRESS;
                    end
                end                
                DATA : begin
                    if(data_counter == 7) begin
                        data_counter <= 0;
                        cs_value <= 1;
                        state <= STOP;
                    end
                    else begin
                        data_counter <= data_counter + 1;
                        state <= DATA;
                    end
                end
                STOP : begin 
                    state <= IDLE;
                end
            endcase
     end
end

   always @ (posedge SCLK) begin
            case(state)
            IDLE : begin
                led_debug[0] = 1;
                data_write_buffer_instruction <= 8'b0000_1001;
                data_write_buffer_address <= 8'b0000_1011;
            end
            INSTRUCTION : begin
             led_debug[1] = 1;
                data_write_buffer_instruction <= (cs_value == 1) ? 8'b0000_1001 : {data_write_buffer_instruction[6:0],1'b1 };
                MOSI <= data_write_buffer_instruction[7];
            end 
            ADDRESS : begin
             led_debug[2] = 1;
                data_write_buffer_address <= (cs_value == 1) ? 8'b0000_1011 : {data_write_buffer_address[6:0],1'b1 };
                MOSI <= data_write_buffer_address[7];
            end
            DATA : begin
             led_debug[3] = 1;
                data_read_buffer <= {data_read_buffer[6:0],MISO};
                MOSI <= 0;
             end
             STOP : begin
              led_debug[4] = 1;
                Received_Data <= data_read_buffer;
             end
            endcase
end

        
       wire [7:0] value_data;   
       bin_to_dec adc_bcd(.bin(Received_Data),  .bcd(value_data));
                                                //0부터 1023까지
       fnd_cntr (.clk(clk), .reset_p(reset_p), .value(value_data), .com(com), .seg_7(seg_7));
    
endmodule














module Clock_Divider (
    input clk,
    input reset,
    output reg SCLK = 0
);
    reg [3:0] SCLK_count = 0;

    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            SCLK <= 0;
            SCLK_count <= 0;
        end
        else begin
            if(SCLK_count == 10) begin
                SCLK_count <= 0;
                SCLK <= ~SCLK;
            end
            else begin
                SCLK_count <= SCLK_count + 1;
            end
        end
       
    end
endmodule