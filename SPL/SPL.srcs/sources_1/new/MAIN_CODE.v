`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_ADDR_DATA(
    input clk,reset_p,
    input miso, //MASTER IN SLAVE OUT,
    input [1:0] btn,
    output reg sck, //SPI Clock
    output reg mosi,    //MASTER OUT SLAVE IN
    /*output reg [7:0] received_data, */
    output reg [15:0] led_debug,
    output [3:0] com,
    output [7:0] seg_7);
    
    reg [7:0] received_data;

    assign rst = reset_p;       //그냥 리셋임
    
    wire start_sck;

     wire btn_pedge;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge));   
 
     reg [7:0] state_read_data;   

         
    reg[2:0] count_5ns;  //us초
    reg sck_clk_enable;
    always@(posedge clk or posedge reset_p)begin
        if(reset_p)begin 
            count_5ns = 0;
            sck = 1;
        end
        else if(sck_clk_enable)begin
            if(clk)begin
                if(count_5ns >=4)begin
                    count_5ns = 0;
                    sck = ~sck;
                end
                else count_5ns = count_5ns + 1;
            end
        end
        else if(!sck_clk_enable)begin
            sck = 1;
            count_5ns = 0;
        end
    end      

    //SCK
    wire sck_p_edge, sck_n_edge;
    edge_detector_n(.clk(clk), .reset_p(reset_p) ,.cp(sck), .n_edge(sck_n_edge), .p_edge(sck_p_edge));
    
    
    
    // data_mode + addr값 지정해주기
    reg [7:0] addr;
    reg [7:0] data;  
    parameter modereg_addr = 8'h11;
    parameter modereg_data  = 8'h3f;
    parameter comreg_addr   = 8'h01;
    parameter comreg_data   = 8'h0c;
    parameter interg_addr     = 8'h02;  
    parameter interg_data     = 8'h30;
    
    /*
    parameter modereg_reset = 8'h3f;
    */
    
    reg [5:0] state, next_state;
    parameter IDLE = 6'b00_0001;      
    parameter COUNT_ADDR_DATA = 6'b00_0010;
    parameter SEND_ADDR = 6'b00_0100;           //클럭 신호
    parameter SEND_DATA = 6'b00_1000;
    parameter READ_DATA = 6'b01_0000;
    parameter STOP = 6'b10_0000;


    reg [2:0] count_bit;
    reg [1:0] addr_data_count;

     
    //MAIN
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state =IDLE;
        else state =next_state;
    end
    
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = IDLE;
            addr_data_count =0;
            count_bit =7;
            led_debug =0;
            sck_clk_enable =0;
            received_data = 8'b0;
        end
        else begin
            case(state)
                IDLE : begin
                sck_clk_enable =0;
                if(btn_pedge) begin
                    sck_clk_enable =1;
                    led_debug[0] =1;         
                    next_state =  COUNT_ADDR_DATA;  
                    end
                end    
                
                COUNT_ADDR_DATA : begin
                led_debug[1] =1;
                    if(addr_data_count<3) next_state = SEND_ADDR;
                    else next_state = READ_DATA;
                end
                
                SEND_ADDR : begin
                led_debug[2] =1;
                    case (addr_data_count)
                        0: addr <= modereg_data;
                        1: addr <= comreg_data;
                        2: addr <= interg_data;
                    endcase                     
                         
                    //addr의 count_bit를 mosi에 넣는 코드
                     if(sck_p_edge) begin  
                        mosi <=addr[count_bit]; 
                     end
                     else if(sck_n_edge) begin 
                        if(count_bit ==0) begin 
                            count_bit =7;
                            next_state =SEND_DATA;  
                        end
                        else count_bit = count_bit -1; 
                     end                                         
                 end       
                 
                
                SEND_DATA : begin
                led_debug[3] =1;
                    //addr_data_count에 따른 addr값 코드
                    case (addr_data_count)
                        0: data <= modereg_data;
                        1: data <= comreg_data;
                        2: data <= interg_data;
                    endcase
                     
                 
                   if(sck_p_edge) begin
                        mosi <= data[count_bit] ; 
                   end
                   else if(sck_n_edge) begin
                            if(count_bit ==0) begin 
                              count_bit =7;
                              next_state =COUNT_ADDR_DATA;  
                              addr_data_count = addr_data_count +1;
                            end
                            else count_bit = count_bit -1; 
                     end                                         
                end                             
                //1 . mosi에다가 주소랑 데이터를 보내면 miso 값이 생성된다.
                //2. mosi에 있는 데이터값을 miso로 보낸다.
                //3. 
                
                READ_DATA : begin
                
                    led_debug[4] =1;
                   state_read_data <= {state_read_data[6:0],miso};     
                              /*      
                    state_read_data[c3ount_bit] <= miso;
                              if(count_bit ==0) begin 
                                 count_bit =7;
                                 */
                                 next_state =STOP;  
                      /*      end
                            else begin
                                count_bit <= count_bit - 1;
                            end*/
                end
                
                
                STOP : begin
                        received_data = state_read_data;
                        led_debug[5] =1;
                        next_state = IDLE;
                    end

            endcase
        end
    end

       wire [7:0] value_data;   
       bin_to_dec adc_bcd(.bin(received_data),  .bcd(value_data));
                                                //0부터 1023까지
       fnd_cntr (.clk(clk), .reset_p(reset_p), .value(value_data), .com(com), .seg_7(seg_7));    

endmodule