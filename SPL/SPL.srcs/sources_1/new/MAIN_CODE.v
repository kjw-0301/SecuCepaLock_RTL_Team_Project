`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_MASTER(
    input clk,reset_p,
    input miso,
    output reg sck,
    output mosi,
    output [15:0]led_debug);
    
    // 1. NSS 처리
    // 2, INTERRUPUT발생시키려면 신호에 의해 발생하는건데 그 신호를 어떻게 잡을면 좋을까
    // 3. parameter 더 보완할거 있나.
    //4. 찍었을떄 주소값 처리 + 데이터 처리는 ?
    
    
    
    
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
            state = IDLE;
                nss =1;
        end
        else begin
            case(state)
                 IDLE  : begin // -> 처음엔 interruput    그 이후엔 MOSI로
                    nss= 1;
                 end
                 
                 INTERRUPUT : begin

                 end
                 
                 SEND_MOSI : begin
                    nss= 0;
                 end
                 
                 SEND_MISO : begin
                    nss =0;
                 end
                 
            endcase
        end
    end
    
    

endmodule