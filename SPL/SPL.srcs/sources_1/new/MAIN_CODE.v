`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_ADDR(
    input clk,reset_p,
    input miso, //MASTER IN SLAVE OUT,
    input irq,
    input[3:0] btn,
    output reg sck, //SPI Clock
    output reg mosi,    //MASTER OUT SLAVE IN
    output [15:0]led_debug);
   
   
    assign rst = reset_p;       //그냥 리셋임
    
    
    wire start_stop;
    wire clk_start;
    assign clk_start = start_stop ? clk : 0;
    button_cntr(.clk(clk), .reset_p(reset_p),.btn(btn[1]),.btn_nedge(start_stop));
        
   //CLOCK US       
    wire clk_50ns;  //us초
    clock_div_10(.clk(clk_start), .reset_p(reset_p),.clk_div_10_nedge(clk_50ns)); 
    //SCK
    wire sck_p_edge, sck_n_edge;
    edge_detector_n(.clk(clk_start), .reset_p(reset_p) ,.cp(clk_50ns), .n_edge(sck_n_edge), .p_edge(sck_p_edge));    
    
    // data_mode + addr값 지정해주기
    reg[2:0] rc_counter;
    reg [7:0] addr;
    reg [7:0] data;   
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) rc_counter = 0;
        case(rc_counter)
            1:begin
                addr = 8'h11; 
                data = 8'h3f;
            end
            2:begin
                addr = 8'h01; 
                data = 8'h0c;
            end
            3:begin
                addr = 8'h02; 
                data = 8'h30;
            end
        endcase
    end

    reg [2:0] count_bit;
    /*
    reg [3:0] command;
    */
    
    /*parameter modereg = 8'h11;  */    
  
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
                    if(rc_counter <= 0) rc_counter = rc_counter + 1;
                    else if(rc_counter > 3) rc_counter = 1;
                    
                    if(sck_p_edge)  mosi =addr[count_bit]; 
                        if(sck_n_edge) begin
                            if(count_bit == 0) begin 
                                count_bit = 7;
                                next_state = SEND_DATA;  
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
                    if(rc_counter < 3) next_state = SEND_ADDR; 
                    else if(rc_counter > 3) next_state = IDLE;
                end
            endcase
        end
    end


endmodule
//=============================================================
