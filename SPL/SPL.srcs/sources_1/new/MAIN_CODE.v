`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_MASTER(
    input clk,reset_p,
    input miso,
    input irq, //(카드키 데이터)
    output reg sck,
    output mosi,
    output [15:0]led_debug);
    
    
    reg [3:0] state, next_state;
    parameter IDLE =4'b0_001;
    parameter INTERRUPUT =4'b0_010;
    parameter SEND_MOSI =4'b0_100;
    parameter SEND_MISO =4'b1_000;
    
    //CLOCK(1NS)
    wire clk_div_10_nedge;
    clock_div_10(.clk(clk), .reset_p(reset_p),.clk_div_10_nedge(clk_div_10_nedge));
    
    
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state =IDLE;
        else state =next_state;
    end
    
    reg nss;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = IDLE;
                nss =0;
        end
        else begin
            case(state)
                 IDLE  : begin // -> 처음엔 interruput    그 이후엔 MOSI로
                    next_state = INTERRUPUT;
                 end
                 
                 INTERRUPUT : begin
                        if(!irq) begin
                            next_state =SEND_MOSI;
                        end
                 end
                 
                 SEND_MOSI : begin
                 end
                 
                 SEND_MISO : begin
                        next_state =IDLE;
                 end
                 
            endcase
        end
    end
    
    assign led_debug[3:0] = state;

endmodule