`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module SPI_ADDR(
    input clk,reset_p,
    input wire miso, //MASTER IN SLAVE OUT,
    input irq,
    input[3:0] btn,
    output reg sck, //SPI Clock
    output reg mosi,    //MASTER OUT SLAVE IN
    output rst,
    output reg nss,
    output reg [15:0] led_debug);
   
    reg [7:0] received_data;  
    reg [5:0] state, next_state;
    parameter IDLE                  = 9'b0_0000_0001;      
    parameter COUNT_ADDR_DATA       = 9'b0_0000_0010;
    parameter SEND_ADDR             = 9'b0_0000_0100;   //클럭 신호
    parameter SEND_DATA             = 9'b0_0000_1000;
    parameter READ_DATA             = 9'b0_0001_0000;
    parameter STAY_REQA_FOR_TAG     = 9'b0_0010_0000;
    parameter ATQA_RECEIVED_ADDR    = 9'b0_0100_0000;
    parameter ATQA_RECEIVED_DATA    = 9'b1_0000_0000;
 
   
    parameter commandReg_addr       = 8'h02;
    parameter modeReg_addr          = 8'h11;
    parameter TxModeReg_addr        = 8'h12;
    parameter TxcontrolReg_addr     = 8'h14; 
    parameter TxASKReg_addr         = 8'h15;
    parameter ComIrqReg_addr        = 8'h04;
    parameter bitframingReg_addr    = 8'h0D;
    ///////////////////////////////////////// - init    
    parameter FIFODataReg_addr      = 8'h09;     
    //parameter commandReg_TRANS_addr = 8'h01;
    
    parameter commandReg_data       = 8'h00;
    parameter modeReg_data          = 8'hc0;
    parameter TxModeReg_data        = 8'hc0;
    parameter txcontrolReg_data     = 8'h03;
    parameter TxASKReg_data         = 8'h40;
    parameter ComIrqReg_data        = 8'h04;
    parameter bitframingReg_data    = 8'h07;
 ///////////////////////////////////////// - init
    parameter FIFODataReg_REQA_data = 7'h26;
    //parameter TRANSCEIVE_data       = 8'h0C;

    assign rst = reset_p;       //그냥 리셋임
    
    wire start_sck;
    button_cntr btn_sck(.clk(clk), .reset_p(reset_p),.btn(btn[1]),.btn_nedge(start_sck));
    
    reg[2:0] count_microsec5;
    wire clk_usec;
    clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));
  
         
    reg[3:0] count_80ns;  //us초
    reg sck_clk_enable;
    always@(posedge clk or posedge reset_p)begin
        if(reset_p)begin 
            count_80ns = 0;
            sck = 1;
        end
        else if(sck_clk_enable)begin
            if(clk)begin
                if(count_80ns >=7)begin
                    count_80ns = 0;
                    sck = ~sck;
                end
                else count_80ns = count_80ns + 1;
            end
        end
        else if(!sck_clk_enable)begin
            sck = 1;
            count_80ns = 0;
        end
    end      
   

    //SCK
    wire sck_p_edge, sck_n_edge;
    edge_detector_n(.clk(clk), .reset_p(reset_p) ,.cp(sck), .n_edge(sck_n_edge), .p_edge(sck_p_edge));
    
    // data_mode + addr값 지정해주기

    reg [7:0] addr;
    reg [7:0] data;  
    reg [2:0] count_bit;
    reg [3:0] addr_data_count;
    
    
     
    //MAIN
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state =IDLE;
        else state =next_state;
    end
    
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            count_microsec5 = 0;
            next_state = IDLE;
            addr_data_count =0;
            count_bit =7;
            led_debug =0;
            sck_clk_enable =0;
            received_data = 8'b0;
            nss =1;
        end
        else begin
            case(state)
                IDLE : begin  
                    nss = 1;
                    sck_clk_enable =0;
                    if(start_sck)begin
                        led_debug[0] =1;        
                        nss = 0;
                        sck_clk_enable = 1;
                        next_state = COUNT_ADDR_DATA;  
                    end                          
                end    
                
                COUNT_ADDR_DATA : begin
                    led_debug[1] =1;
                    if(addr_data_count < 11) next_state = SEND_ADDR;
                    else next_state = READ_DATA;
                end
                
                SEND_ADDR : begin
                led_debug[2] =1;
                    case (addr_data_count)
                        0: addr <= {1'b1,commandReg_addr,1'b0};
                        1: addr <= {1'b1,modeReg_addr,1'b0};
                        2: addr <= {1'b1,TxModeReg_addr,1'b0};
                        3: addr <= {1'b1,TxcontrolReg_addr,1'b0};
                        4: addr <= {1'b1,TxASKReg_addr,1'b0};
                        5: addr <= {1'b1,ComIrqReg_addr,1'b0};
                        6: addr <= {1'b1,bitframingReg_addr,1'b0};
                        7: addr <= {1'b1,FIFODataReg_addr,1'b0};
                    endcase                     
                         
                    //addr의 count_bit를 mosi에 넣는 코드
                     if(sck_p_edge) begin  
                        mosi <=addr[count_bit]; 
                     end
                     else if(sck_n_edge) begin 
                        if(count_bit ==0) begin 
                            count_bit =7;
                            next_state = SEND_DATA;
                        end
                        else count_bit = count_bit -1; 
                     end                                        
                 end       
                 
                
                SEND_DATA : begin
                led_debug[3] =1;
                    //addr_data_count에 따른 addr값 코드
                    case (addr_data_count)
                        0: data <= commandReg_data;
                        1: data <= modeReg_data;        
                        2: data <= TxModeReg_data;       
                        3: data <= txcontrolReg_data;    
                        4: data <= TxASKReg_data;        
                        5: data <= ComIrqReg_data;       
                        6: data <= bitframingReg_data;   
                        7: data <= FIFODataReg_REQA_data;
                    endcase                         
                    if(sck_p_edge) begin
                        mosi <= data[count_bit] ; 
                    end
                    else if(sck_n_edge) begin 
                        if(count_bit ==0) begin 
                            count_bit =7;
                            addr_data_count = addr_data_count + 1;
                            if(addr_data_count == 7) next_state = STAY_REQA_FOR_TAG;
                            else if(addr_data_count > 7)next_state =COUNT_ADDR_DATA;  
                        end
                        else count_bit = count_bit -1; 
                    end
                end
                  
                    
                STAY_REQA_FOR_TAG: begin
                    led_debug[4] =1;
                    if(addr_data_count == 7)begin
                        if(sck_p_edge )mosi <=8'h09;
                        else if(sck_n_edge) begin 
                            if(count_bit ==0) begin 
                                count_bit =7;
                                if(clk_usec)begin
                                    if(count_microsec5 >= 4)begin
                                        mosi <= 7'h26;
                                        count_microsec5 = 0; 
                                    end
                                    else count_microsec5 = count_microsec5 + 1;
                                end
                            end
                            else count_bit = count_bit -1;
                        end
                        if(miso)begin
                            next_state =  ATQA_RECEIVED_ADDR;
                        end
                    end
                    
                end
                
                ATQA_RECEIVED_ADDR : begin
                led_debug[5] =1;
                    case (addr_data_count)
                        0: addr <= {1'b1,commandReg_addr,1'b0};
                        1: addr <= {1'b1,modeReg_addr,1'b0};
                        2: addr <= {1'b1,TxModeReg_addr,1'b0};
                        3: addr <= {1'b1,TxcontrolReg_addr,1'b0};
                        4: addr <= {1'b1,TxASKReg_addr,1'b0};
                        5: addr <= {1'b1,ComIrqReg_addr,1'b0};
                        6: addr <= {1'b1,bitframingReg_addr,1'b0};
                        7: addr <= {1'b1,FIFODataReg_addr,1'b0};
                        8: addr <= {1'b1,FIFODataReg_addr,1'b0};
                        9: addr <= {1'b1,commandReg_addr,1'b0};
                        10: addr <= {1'b1,bitframingReg_addr,1'b0};
                    endcase                     
                         
                    //addr의 count_bit를 mosi에 넣는 코드
                     if(sck_p_edge) begin  
                        mosi <=addr[count_bit]; 
                     end
                     else if(sck_n_edge) begin 
                        if(count_bit ==0) begin 
                            count_bit =7;
                            if(clk_usec)begin
                                count_microsec5 = count_microsec5 + 1;
                                if(count_microsec5 >= 1)begin
                                    sck_clk_enable = 1;
                                    count_microsec5 = 0;
                                    next_state =ATQA_RECEIVED_DATA;  
                                end
                                else sck_clk_enable = 0;
                            end
                        end
                        else count_bit = count_bit -1; 
                     end                                         
                 end       
                 
                
                ATQA_RECEIVED_DATA : begin
                led_debug[6] =1;
                    //addr_data_count에 따른 addr값 코드
                    case (addr_data_count)
                        0: data <= commandReg_data;
                        1: data <= modeReg_data;        
                        2: data <= TxModeReg_data;       
                        3: data <= txcontrolReg_data;    
                        4: data <= TxASKReg_data;        
                        5: data <= ComIrqReg_data;       
                        6: data <= bitframingReg_data;   
                        7: data <= FIFODataReg_REQA_data;
                        8: data <= {8'h93,8'h20};
                        9: data <= {8'h0c};
                        10: data <= {1'b1<<7};
                    endcase                         
                    if(sck_p_edge) begin
                        mosi <= data[count_bit] ; 
                    end
                    else if(sck_n_edge) begin 
                        if(count_bit ==0) begin 
                            count_bit =7;
                            if(clk_usec)begin
                                count_microsec5 = count_microsec5 + 1;
                                if(count_microsec5 >= 1)begin
                                    count_microsec5 = 0;
                                    addr_data_count = addr_data_count + 1;
                                    
                                    if(addr_data_count >= 11) next_state = IDLE;
                                    else if(addr_data_count < 11)next_state =ATQA_RECEIVED_ADDR;  
                                end
                            end
                        end
                        else count_bit = count_bit -1; 
                     end                                                           
                end
            endcase
        end
    end
endmodule

