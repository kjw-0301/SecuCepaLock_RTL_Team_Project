`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_MASTER(
    input clk,reset_p,
    input miso,
    output reg sck,
    output mosi,
    output [15:0]led_debug);
    
    // 1. NSS ó��
    // 2, INTERRUPUT�߻���Ű���� ��ȣ�� ���� �߻��ϴ°ǵ� �� ��ȣ�� ��� ������ ������
    // 3. parameter �� �����Ұ� �ֳ�.
    //4. ������� �ּҰ� ó�� + ������ ó���� ?
    
    
    
    
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
                 IDLE  : begin // -> ó���� interruput    �� ���Ŀ� MOSI��
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