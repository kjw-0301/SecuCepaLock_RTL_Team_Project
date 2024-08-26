`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_ADDR(
    input clk,reset_p,
    input miso, //MASTER IN SLAVE OUT,
    input irq,
    output reg sck, //SPI Clock
    output reg mosi,    //MASTER OUT SLAVE IN
    output rst,
    output [15:0]led_debug);
   
   
    assign rst = reset_p;       //그냥 리셋임
    
    
    wire start_stop;
    wire clk_start;
    assign clk_start = start_stop ? clk : 0;
        
   //CLOCK US       
    wire clk_div_10_nedge;  //us초
    clock_div_10(.clk(clk_start), .reset_p(reset_p),.clk_div_10_nedge(clk_div_10_nedge)); 
    
    
    //SCK
    wire sck_p_edge, sck_n_edge;
    edge_detector_n(.clk(clk_start), .reset_p(reset_p) ,.cp(sck), .n_edge(sck_n_edge), .p_edge(sck_p_edge));
    
    
    
    // data_mode + addr값 지정해주기

    reg [7:0] addr;
    always @(posedge clk or posedge reset_p) begin
        case(mode)

        endcase
    end





    reg [2:0] count_bit;
    /*
    reg [3:0] command;
    */
    
    
    /*parameter modereg = 8'h11;  */
     parameter modereg_reset = 8'h3f;
    

    wire [7:0] data;   
   
    
    reg [2:0] state, next_state;
    parameter IDLE = 4'b0_001;       
    parameter SEND_ADDR = 4'b0_010;   //클럭 신호
    parameter SEND_DATA = 4'b0_100;
    parameter STOP = 4'b1_000;


    //MAIN
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state =IDLE;
        else state =next_state;
    end
    
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            state = IDLE;
        end
        else begin
            case(state)
                IDLE : begin
                    if(start_stop) begin
                        next_state = SEND_ADDR;
                    end
                end
                
                SEND_ADDR : begin
                    if(sck_p_edge)  mosi =addr[count_bit]; 
                        if(sck_n_edge) begin
                            if(count_bit ==0) begin 
                              count_bit =7;
                              next_state =SEND_DATA;  
                              end
                              
                        else count_bit = count_bit -1; 
                    end                                         
                end       
                
                
                SEND_DATA : begin
                    if(sck_p_edge)  mosi =data[count_bit]; 
                        if(sck_n_edge) begin
                            if(count_bit ==0) begin 
                              count_bit =7;
                              next_state =STOP;  
                              end
                        else count_bit = count_bit -1; 
                    end                                         
                end                             

                STOP : begin
                    if(!start_stop) begin
                        next_state = IDLE;
                    end
                end

            endcase
        end
    end


endmodule
//=============================================================
