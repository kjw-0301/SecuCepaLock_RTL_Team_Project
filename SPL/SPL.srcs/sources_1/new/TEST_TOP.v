`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_TEST_TOP(
    input clk,reset_p,
    input miso, //MASTER IN SLAVE OUT,
    input irq,
    output reg sck, //SPI Clock
    output mosi,    //MASTER OUT SLAVE IN
    output [15:0]led_debug);
   
    //MAIN parameter
    reg [3:0] state, next_state;
    parameter IDLE =4'b0_001;                   //전체 초기화
    parameter INTERRUPUT =4'b0_010;     //MOSI로 사용할 reg주소와 명령어, 실행시킬 bit활성화
    parameter SEND_MOSI =4'b0_100;       
    parameter SEND_MISO =4'b1_000;
    
    //SPI parameter
    reg [7:0]  irq_data = 8'h38;
    
    
    edge_detector_n(clk, reset_p,cp,  p_edge, n_edge);
    
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
        end
        else begin
            case(state)
                 IDLE  : begin
                    next_state = INTERRUPUT;
                 end
                 
                 INTERRUPUT : begin
                       next_state = SEND_MOSI;  
                       
                 end
                 
                 SEND_MOSI : begin
                    
                    next_state = SEND_MISO;               

                 end
                 
                 SEND_MISO : begin
                    next_state = IDLE; 
                 end
                 
            endcase
        end
    end
    
    assign led_debug [3:0] = state;
    
endmodule