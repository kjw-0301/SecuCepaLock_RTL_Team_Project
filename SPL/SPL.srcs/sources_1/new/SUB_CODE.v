`timescale 1ns / 1ps
//==============================================
module clock_div_10(
       input clk, reset_p,
       output clk_div_10,     
       output clk_div_10_nedge);
       
       reg  [3:0] cnt_sysclk;       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_sysclk =0;
          
          else  begin
              if(cnt_sysclk >= 9) cnt_sysclk = 0; 
             else cnt_sysclk = cnt_sysclk +1;
            
            end  
       end   
                assign clk_div_10 = (cnt_sysclk < 5) ? 0 : 1;     
                
    edge_detector_n ed1(  
        .clk(clk),
        .reset_p(reset_p), 
        .cp(clk_div_100),  
        .n_edge(clk_div_100_nedge));
      
    
endmodule

//=================================================
module edge_detector_n(
    input clk, reset_p,
    input cp, 
    output p_edge, n_edge);
    
    reg ff_cur, ff_old;
    always @(negedge clk or posedge reset_p) begin
        if (reset_p) begin
            ff_cur <= 0;
            ff_old <= 0;
        end
        else begin
            ff_old <= ff_cur; 
            ff_cur <= cp; 

        end
    end
    
    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1 : 0;
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1 : 0;
    
endmodule

//======================================================
module fnd_cntr(     //��Ʈ�ѷ�  //fnd 0�϶� ������.  
        input clk, reset_p,
        input [15:0] value,
        output [3:0] com,       //�������  //LED�� ���� ���
        output [7:0] seg_7);
        
        ring_counter_fnd rc(clk, reset_p, com);
         // ����         �ν��Ͻ���
        reg [3:0] hex_value;     //decoder_7seg�� �� reg
        always @(posedge clk) begin
                case(com)  
                    4'b1110 : hex_value = value[3:0];   
                    4'b1101 : hex_value = value[7:4];   
                    4'b1011 : hex_value = value[11:8];   
                    4'b0111 : hex_value = value[15:12];   
            endcase
        end
        
        decoder_7seg sub7seg(.hex_value(hex_value), .seg_7(seg_7));
endmodule

//=============================================================
//��ư ��Ʈ�ѷ�
//=============================================================
module button_cntr(
        input clk, reset_p,
        input btn,
        output btn_pedge, btn_nedge);
        
         reg[20:0] clk_div = 0; 
         always @(posedge clk)clk_div = clk_div +1;
       
       
         //ä�͸� ���� ��ŸƮ
         wire clk_div_nedge;  
    edge_detector_n ed(.clk(clk), .reset_p(reset_p),
                                               .cp(clk_div[16]), .n_edge(clk_div_nedge)); 
                      
                      
         //ä�͸� ��� ����                         
         reg debounced_btn;
         always @(posedge clk or posedge reset_p)begin
            if(reset_p) debounced_btn =0;
            else if(clk_div_nedge) debounced_btn = btn;
         end
         
         

    edge_detector_n btn1(.clk(clk), .reset_p(reset_p),
                               .cp(debounced_btn), .n_edge(btn_nedge), .p_edge(btn_pedge)); 
endmodule

//===============================================================
module ring_counter_fnd(
        input clk, reset_p,
        output reg [3:0] com);
        
            reg[20:0] clk_div = 0;     

    always @(posedge clk)clk_div = clk_div +1;  

           wire clk_div_nedge;  
      
    edge_detector_n ed(.clk(clk), 
                                              .reset_p(reset_p), 
                                              .cp(clk_div[16]),
                                              .n_edge(clk_div_nedge));  
           
        always @(posedge clk or posedge reset_p) begin
            if (reset_p) com = 4'b1110;
            else if(clk_div_nedge)begin
                if (com == 4'b0111) com = 4'b1110;
                else com[3:0] = {com[2:0], 1'b1};
            end
        end
    
endmodule

//==============================================================
module bin_to_dec(
        input [11:0] bin,
        output reg [15:0] bcd
    );

    reg [3:0] i;

    always @(bin) begin
        bcd = 0;
        for (i=0;i<12;i=i+1)begin
            bcd = {bcd[14:0], bin[11-i]};
            if(i < 11 && bcd[3:0] > 4) bcd[3:0] = bcd[3:0] + 3;
            if(i < 11 && bcd[7:4] > 4) bcd[7:4] = bcd[7:4] + 3;
            if(i < 11 && bcd[11:8] > 4) bcd[11:8] = bcd[11:8] + 3;
            if(i < 11 && bcd[15:12] > 4) bcd[15:12] = bcd[15:12] + 3;
        end
    end
endmodule

//======================================================
module decoder_7seg(
    input [3:0] hex_value,
    output reg [7:0] seg_7);
    
    always@(hex_value) begin
          case(hex_value)
                0 : seg_7 = 8'b0000_0011;   //0         
                1 : seg_7 = 8'b1001_1111; 
                2 : seg_7 = 8'b0010_0101; 
                3 : seg_7 = 8'b0000_1101; 
                4 : seg_7 = 8'b1001_1001; 
                5 : seg_7 = 8'b0100_1001; 
                6 : seg_7 = 8'b0100_0001; 
                7 : seg_7 = 8'b0001_1011; 
                8 : seg_7 = 8'b0000_0001; 
                9 : seg_7 = 8'b0001_1001; 
              10 : seg_7 = 8'b0001_0001;  //A 
              11 : seg_7 = 8'b1100_0001;  //b
              12 : seg_7 = 8'b0110_0011;  //C
              13 : seg_7 = 8'b1000_0101;  //D
              14 : seg_7 = 8'b0110_0001;  //E
              15 : seg_7 = 8'b0111_0001;  //F
          endcase
    end

endmodule
//=====================================================================
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// T�ø��÷� (��¿���)
/////////////////////////////////////////////////////////////////////////////////////////////////////////
module T_flip_flop_positive(
    input clk, reset_p,
    input t,
    output reg q
);
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) q = 0;
        else begin
            if (t) q = ~q; // t�� 1�̸� q�� ����
            else q = q; // ���� ����
        end 
    end
     
endmodule
//=======================================================================
module clock_div_100(
       input clk, reset_p,
       output clk_div_100,     
       output clk_div_100_nedge);
       
       reg  [6:0] cnt_sysclk;       //sysclk :�ý��� Ŭ��   10-6
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_sysclk =0;
          
          else  begin
              if(cnt_sysclk >= 99) cnt_sysclk = 0; 
             else cnt_sysclk = cnt_sysclk +1;
            
            end  
       end   
                assign clk_div_100 = (cnt_sysclk < 50) ? 0 : 1;     
                
    edge_detector_n ed1(        //�ٿ� ���� �����Ϳ� 100���� Ŭ��+�ٿ��    
                                                         //���� �ڵ忡�� �ҷ�����
        .clk(clk),
        .reset_p(reset_p),    //(������ ck������ 1Ŭ���ڿ� ���´�)
        .cp(clk_div_100),  
        .n_edge(clk_div_100_nedge));
      
    
endmodule

//=======================================================================

module clock_div_60(
       input clk, reset_p,
       input clk_source,            //1us clock
       output clk_div_60,     //100����(��)
       output clk_div_60_nedge);
       
       reg  [9:0] cnt_clksource;       //sysclk :�ý��� Ŭ��       10-9
     //�����ϸ� �÷��������� ��ġ�� ���̴°� �����̴�.
     // ��ĥ���� 5�� �ٲٸ� �ȴ�.
     // �ַ� integer cnt_clksource�� ���⵵�Ѵ� (32��Ʈ)
     // 32��Ʈ�� �Ѿ�� ������ ���⵵ ����������Ѵ�.
     
     
       wire clk_source_nedge;         //���� ������ �ϰ����� �ҷ�����
               edge_detector_n ed1(.clk(clk), 
                                                        .reset_p(reset_p),    
                                                        .cp(clk_source),  
                                                        .n_edge(clk_source_nedge));
       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_clksource =0;       //�ʱⰪ ����
          
          else  if(clk_source_nedge) begin        //clk_source�� 1���ö�����
                   if(cnt_clksource >= 59) cnt_clksource = 0; 
               else cnt_clksource = cnt_clksource +1;         //1us�� 1�������ؼ� 1ms��
            
            end  
       end   
                assign clk_div_60 = (cnt_clksource < 30) ? 0 : 1;     
                
    edge_detector_n ed2(              //�ٿ� ���� �����Ϳ� 100���� Ŭ��+�ٿ��
        .clk(clk), .reset_p(reset_p),    //(������ ck������ 1Ŭ���ڿ� ���´�)
        .cp(clk_div_60),  .n_edge(clk_div_60_nedge));
    
endmodule

//==============================================================================

module clock_div_1000(
       input clk, reset_p,
       input clk_source,            //1us clock
       output clk_div_1000,     //100����(��)
       output clk_div_1000_nedge);
       
       reg  [9:0] cnt_clksource;       //sysclk :�ý��� Ŭ��       10-9
     
       wire clk_source_nedge;         //���� ������ �ϰ����� �ҷ�����
               edge_detector_n ed1(.clk(clk), 
                                                        .reset_p(reset_p),    
                                                        .cp(clk_source),  
                                                        .n_edge(clk_source_nedge));
       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_clksource =0;       //�ʱⰪ ����
          
          else  if(clk_source_nedge) begin        //clk_source_nedge�� 1���ö�����
                   if(cnt_clksource >= 999) cnt_clksource = 0; 
               else cnt_clksource = cnt_clksource +1;         //1us�� 1�������ؼ� 1ms��
            
            end  
       end   
                assign clk_div_1000 = (cnt_clksource < 500) ? 0 : 1;     
                
    edge_detector_n ed2(              //�ٿ� ���� �����Ϳ� 100���� Ŭ��+�ٿ��
        .clk(clk), .reset_p(reset_p),    //(������ ck������ 1Ŭ���ڿ� ���´�)
        .cp(clk_div_1000),  .n_edge(clk_div_1000_nedge));
    
endmodule
//==============================================================
//�ε尡 ������ 60�� ī���� (156~160)
//==============================================================
module loadable_counter_bcd_60(
      input clk,reset_p,
      input clk_time,
      input load_enable,
      input [3:0] load_bcd1, load_bcd10,
      output reg [3:0] bcd1, bcd10); //1�� �ڸ� , 10�� �ڸ�
      
      wire clk_time_nedge;
          edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p),   
                                     .cp(clk_time),  .n_edge(clk_time_nedge));
      
      always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin       //���� �� 
            bcd1 = 0;
            bcd10= 0;
        end
        else begin
                            //������
                            //bcd1�� 10�� �ε�Ȱ��� �ٽ� �ִ´�.
                    if(load_enable)begin
                        bcd1 = load_bcd1;
                        bcd10 = load_bcd10;
                    end   
                        else if(clk_time_nedge) begin
                            if(bcd1 >= 9) begin
                                bcd1 =0;
                          if(bcd10 >= 5) bcd10 = 0;       //60���ִϱ� 60�̸� 0���� ����
                                else bcd10 = bcd10 +1;
                            end
                              else  bcd1 = bcd1 +1;
          end
         end
      end
endmodule        

//===============================================================

module I2C_controller(
    input clk,reset_p,
    input [6:0] addr,
    input rd_wr,        //�ּ� �аų� ���ų�
    input [7:0] data,
    input comm_go,  //�̰� 0->1������ ��Ž���
    output reg  scl, sda,
    output reg [15:0] led_debug);
     

    
    
    parameter IDLE =  7'b000_0001;
    parameter  COMM_START=  7'b000_0010;
    parameter  SEND_ADDR=  7'b000_0100;
    parameter  RD_ACK=  7'b000_1000;
    parameter  SEND_DATA=  7'b001_0000;
    parameter  SCL_STOP=  7'b010_0000;      //Ŭ�� ��ž (�ӵ� 100KHZ����) 10us
    parameter  COMM_STOP=  7'b100_0000;


   //CLOCK US       
   wire clk_usec;
    //us��
   clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));     

   //EDGE_DETECTOR  
   wire scl_pedge, scl_nedge, comm_go_pedge;
 
   edge_detector_n SCL_P_N(.clk(clk), .reset_p(reset_p), .cp(scl),  
                                                   .p_edge(scl_pedge), .n_edge(scl_nedge));
  
   edge_detector_n COMM_GO_P(.clk(clk), .reset_p(reset_p), .cp(comm_go),  
                                                         .p_edge(comm_go_pedge));


    //��α� ADDRESS PACKET  FORMAT ����
    //10us��� �ϴµ� en��ȣ�� �߰��� �����̴�.(SCL����)
    reg [2:0] count_usec5;
    reg scl_e;  //clock_enable 
    always @(posedge clk or posedge reset_p) begin
        if(reset_p)  begin
            count_usec5 = 0; 
            scl =1;         //scl�� ó���� 1
        end
        
        else if(scl_e)begin         //scl_e�� scl �� 1�ϋ��� ���� 
            if(clk_usec) begin
                if(count_usec5 >= 4)begin
                    count_usec5 =0;
                    scl = ~scl;
                end
                else  count_usec5 = count_usec5 +1; 
            end
        end   
        
            else if(!scl_e) begin    //scl_e �� 0�̶��
                scl =1;                 //scl�� ������ 1�� ������.
                count_usec5 = 0;
            end   
    end                                               
   
   
   //state, �ּ� ����
    reg [6:0] state ,next_state;     
    wire [7:0] addr_rw;
    assign addr_rw = {addr, rd_wr}; //�ּ�, Read/Write ���ļ� 8��Ʈ 
    
    reg [2:0] cnt_bit;
    reg stop_flag;  //������ ������ ���� 1��(RD_ACK���� ���)
    
    //MAIN CODE(�������� ����)
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state = IDLE;
        else  state = next_state;
    end
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state =IDLE;
            scl_e =0;   //clk �� 0����
            sda =1;
            cnt_bit =7; //�ֻ��� ��Ʈ �������ϱ� 7�� �ʱ�ȭ
            stop_flag =0; 
            led_debug = 0;  
        end
        else begin
            case(state)
                IDLE : begin
                led_debug[0] = 1;
                    scl_e =0;
                    sda =1;
                    if(comm_go_pedge) next_state = COMM_START;   //comm_go�� 1������ �Ѿ
                end
                
                COMM_START : begin  //SCL�� 1�λ��¿��� SDA�� 0���� ������, ��Ž���>>clock������
                    led_debug[1] = 1;
                    sda =0;
                    scl_e =1;   //clock ������ >>5us�Ŀ� 0�� �ȴ�.
                    next_state = SEND_ADDR;
                end 
                
                SEND_ADDR : begin
                led_debug[2] = 1;
                    if(scl_nedge) sda =addr_rw[cnt_bit]; //ó���� 7����Ʈ �ش�.
                    else if(scl_pedge) begin
                        if(cnt_bit ==0) begin 
                            cnt_bit =7;
                            next_state =RD_ACK;  //0�� �Ǹ� next_state��
                        end
                        else cnt_bit = cnt_bit -1; //p_edge���� �ƹ��͵� ���ϴϱ� �״�����Ʈ���� ���� �̸� �غ�
                    end                                          //�� 7���� 0����  �� ���� �ݺ��� �����Ѱ���
                end 
                
                RD_ACK : begin  //RD_ACK������ SEND_DATA�� ���� SCL_STOP���� ���� ���������(2���� ���)
                    led_debug[3] = 1;
                    if(scl_nedge) sda ='bz; 
                    else if(scl_pedge) begin        //pedge������ 1Ŭ�� ��������, clk�� �������ϴϱ� ��
                        if(stop_flag) begin //�ʱ�ȭ ���� 0
                            stop_flag =0;   //SCL ������
                            next_state =SCL_STOP;
                        end
                        else begin
                            stop_flag =1;   // SCL �߰����� 
                            next_state = SEND_DATA;
                        end
                    end
                end 
                
                SEND_DATA : begin
                led_debug[4] = 1;
                if(scl_nedge) sda =data[cnt_bit]; //ó���� 7����Ʈ �ش�.
                    if(scl_pedge) begin
                        if(cnt_bit == 0) begin 
                            cnt_bit =7;
                            next_state =RD_ACK; //0�� �Ǹ� next_state��
                        end
                        else cnt_bit = cnt_bit -1; //p_edge���� �ƹ��͵� ���ϴϱ� �״�����Ʈ���� ���� �̸� �غ�
                    end         
                end 
                
                SCL_STOP : begin
                 led_debug[5] = 1;
                    if(scl_nedge) sda =0;
                    else if(scl_pedge) next_state = COMM_STOP;
                end 
                
                COMM_STOP : begin   //high ���¿��� �����ֵ��� �ش�.
                    led_debug[6] = 1;
                    if(count_usec5 >= 3)begin//3us�� ��ٸ��ٰ�
                        scl_e =0;   //  Ŭ���� high�� �����Ǹ� 
                        sda = 1;
                        next_state =IDLE;
                    end
                end 
            endcase
        end
    end
endmodule

//==============================================================================
//8.21 start

module I2C_lcd_send_byte(
    input clk,reset_p,
    input [6:0] addr,
    input rs, send,     //send�� 1�ϋ� comm_go�� 1������ ���� 4��Ʈ ������/ enable ���� 4��Ʈ������ /enable
    input [7:0] send_buffer,    //data��
    output scl, sda,
    output reg busy,  //busy�� 0�϶��� send�� �������ְ� (�ð��� �� �ɸ��ϱ�)
    output [15:0] led_debug);


    //PARAMETER
                    //NIBBLE= 4��Ʈ
    reg [5:0] state, next_state;
    parameter IDLE                                        = 6'b00_0001; 
    parameter SEND_HIGH_NIBBLE_DISABLE = 6'b00_0010;//���� 4��Ʈ�� �����ǵ� disable�� �ѹ�������
    parameter SEND_HIGH_NIBBLE_ENABLE  = 6'b00_0100; //���� 4��Ʈ�� �����ǵ� enable�� �ѹ�������
    parameter SEND_LOW_NIBBLE_DISABLE  = 6'b00_1000;//���� 4��Ʈ�� �����ǵ� disable�� �ѹ�������
    parameter SEND_LOW_NIBBLE_ENABLE   = 6'b01_0000;//���� 4��Ʈ�� �����ǵ� enable�� �ѹ�������
    parameter SEND_DISABLE                        = 6'b10_0000;//disable��ȣ�� ������ ���� ���а� �Ѵ�.
                          // LCD���� 1����Ʈ(8��Ʈ) �޴°��� 
    
    reg [7:0] data;
    reg comm_go; //�̰� 0->1������ ��Ž���
    I2C_controller(.clk(clk) , .reset_p(reset_p) ,
                           .addr(addr) , .rd_wr(0) , // .rd_wr(0) : ���⸸
                           .data(data), .comm_go(comm_go), 
                           .scl(scl), .sda(sda), .led_debug(led_debug));
    
   //CLOCK US(us��)
   wire clk_usec;
   clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));     
    
    //EDGE_DETECTOR
    wire send_pedge;
    edge_detector_n SCL_P_N(.clk(clk), .reset_p(reset_p), .cp(send), .p_edge(send_pedge));
    

     //COUNT_USEC
    reg[21:0] count_usec;
    reg count_usec_e;   //enable
     always @(negedge clk or posedge reset_p)begin
        if(reset_p) count_usec =0;
          else if(clk_usec && count_usec_e) count_usec = count_usec +1;
          else if(!count_usec_e) count_usec = 0;               
     end  
    
    
    //MAIN CODE
    always @(negedge clk or posedge reset_p)begin
        if(reset_p) state =IDLE;
        else state = next_state;
    end
    
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
             next_state = IDLE;
             busy = 0;
             comm_go =0;
             data = 0;
             count_usec_e =0;
        end
        else begin
            case(state)
                 IDLE : begin
                    if(send_pedge) begin
                        next_state = SEND_HIGH_NIBBLE_DISABLE;
                        busy =1;
                    end
                  end 
                  
                 SEND_HIGH_NIBBLE_DISABLE : begin
                    if(count_usec <= 22'd200)  begin            //200us //start, I2C����(18)  , stop �ؼ� �����Ӱ� 200us
                        data= {send_buffer[7:4], 3'b100, rs};   // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4����)
                        comm_go =1;                                     //BT�� 1�ΰ���
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_HIGH_NIBBLE_ENABLE;   
                    end
                 end 

                 SEND_HIGH_NIBBLE_ENABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C����(18)  , stop �ؼ� �����Ӱ� 200us
                        data= {send_buffer[7:4], 3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4����)
                        comm_go =1;                  //BT�� 1�ΰ���
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_LOW_NIBBLE_DISABLE;   
                    end
                  end 
                 
                 SEND_LOW_NIBBLE_DISABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C����(18)  , stop �ؼ� �����Ӱ� 200us
                        data= {send_buffer[3:0], 3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4����)
                        comm_go =1;                  //BT�� 1�ΰ���
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_LOW_NIBBLE_ENABLE;   
                    end
                 end                      
                 
                 SEND_LOW_NIBBLE_ENABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C����(18)  , stop �ؼ� �����Ӱ� 200us
                        data= {send_buffer[3:0], 3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4����)
                        comm_go =1;                  //BT�� 1�ΰ���
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_DISABLE;   
                        end
                 end
                 
                 SEND_DISABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C����(18)  , stop �ؼ� �����Ӱ� 200us
                        data= {send_buffer[3:0], 3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4����)
                        comm_go =1;                  //BT�� 1�ΰ���
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = IDLE;   
                        busy =0;    //IDLE�� ���� busy�� 1 , ���������� 0
                     end               //�� ������ ���� busy�� 0�̶�� ���� 
                 end
            endcase
          end
      end
endmodule


//================================================================================

module I2C_txtlcd_top(
    input clk,reset_p,
    input [3:0] btn,
    output scl,sda,
    output [15:0] led_debug);
    
    //PARAMETER
    reg [5:0] state, next_state;
    parameter IDLE                       = 6'b00_0001;
    parameter INIT                       = 6'b00_0010;
    parameter SEND_DATA           = 6'b00_0100;
    parameter SEND_COMMAND   = 6'b00_1000;
    parameter SEND_STRING        = 6'b01_0000;
    parameter SEND_COMMAND2 = 6'b10_0000;


    //CLOCK_USEC
    wire clk_usec;  
    clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p),.clk_div_100(clk_usec));
    
    //COUNT_USEC
    reg[21:0] count_usec;
    reg count_usec_e;   //enable
     always @(negedge clk or posedge reset_p)begin
        if(reset_p) count_usec =0;
          else if(clk_usec && count_usec_e) count_usec = count_usec +1;
          else if(!count_usec_e) count_usec = 0;               
     end  
    
    //BTN 
    wire [3:0] btn_pedge;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_pedge[0]));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge[1]));   
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_pedge[2]));   
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(btn_pedge[3]));   
    
    
    //INST
    reg [7:0] send_buffer;      //input�̴ϱ� reg �� �����ؼ� ����(�����͹޴� ��������)
    reg rs,send;
    wire busy;
    I2C_lcd_send_byte(.clk(clk) , .reset_p(reset_p) , .addr(7'h27) ,.rs(rs), .send(send), .send_buffer(send_buffer), 
                                  .scl(scl), .sda(sda), .busy(busy),.led_debug(led_debug));
    
    
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state = IDLE;
        else state = next_state;
    end
    
    
    reg INIT_flag;  //�ߴ��� ���ߴ��� Ȯ�ο�
    reg [2:0]cnt_data;
    reg [5*8-1 :0] hello; //5����(8��Ʈ)¥�� ���� (-1��  0���� �����ϴϱ� ��)([] ���� �� �����)
    reg [3:0] cnt_string; //���ڼ��� ���� ����(���⼱ hello 5���� ��)
    reg [2:0]command2_cnt_data;
    reg [1*8-1 :0]h;
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = IDLE;    
            INIT_flag =0;
            count_usec_e =0;
            cnt_data =0;
            hello = "HELLO";    //���ڴ� �̷������� �ʱ�ȭ( ""�ȿ� hello�� �ƽ�Ű �ڵ� ���� ����)
            h = "H";
            cnt_string =5;
            command2_cnt_data =0;
        end
        else begin
            case(state)
                IDLE : begin            
                    if(INIT_flag) begin
                        if(btn_pedge[0]) next_state = SEND_DATA; //(INIT_flag)�� 1�̸� ��ư0�� ������SEND_BYTE
                        if(btn_pedge[1]) next_state = SEND_COMMAND;
                        if(btn_pedge[2]) next_state = SEND_STRING;
                        if(btn_pedge[3]) next_state = SEND_COMMAND2;
                    end
                    else begin
                        if(count_usec <= 22'd80_000) begin      //80ms
                            count_usec_e =1;
                        end
                        else begin
                             next_state = INIT;     // INIT_flag�� 0�̸� INIT���� ����
                                                             // ó�� �ѹ��� INIT���� ���� �ٽ� ���� ����.
                             count_usec_e =0;
                        end
                    end
                end
                
                INIT : begin    //ó�� �ѹ��� �ϸ� �������.(INIT_flag �� üũ)
                                        //ó���� busy�� 0�̴ϱ� else������ 6���� ���ļ� cnt
                    if(busy) begin  // busy�� send_pedge�� ���Ë� 1�̵ȴ�.
                          send =0;
                          if(cnt_data >= 6) begin
                              next_state = IDLE;
                              INIT_flag = 1;  // �ʱ�ȭ 
                              cnt_data =0;
                              rs =0;
                          end
                      end
                      else if(!send)begin 
                      //send�� 0�� �ƴѰ�쿣 1���� ���ü�������
                      //busy�� send�� ��¿��� ���� �߻��ϰ� clk�� n_edge�� ���� �����ϱ� ������ 
                      // Ŭ�� ���̰� ����� �ִ�. �׷��� ������ send�� 1���� �����ϴ� ��찡 ����� �ְ�
                      //�װ� ���� ���ؼ� !send��� ������ ������ send�� 1�� �������� �����Ѵ�. 
                                 //�� send =0, busy =0 �ϋ�
                          case(cnt_data)  //�ʱ�ȭ �ڵ� 6�� ����
                              0 : send_buffer = 8'h33;  //rs =0;send�� endcase���� 1�� �ָ� sned_buffer
                              1 : send_buffer = 8'h32;  //�� ���� ���·� �Ѿ�°� send_pedge�� ���� �Ѿ�⶧���� 1Ŭ�� �ڿ� �Ѿ
                                                                    // �׷��Ƿ� send = 0 �� �ָ� �ϴ� busy�� 1�� �ƴϹǷ� �Ʒ����� ���󰡰� �ȴ�.
                              2 : send_buffer = 8'h28;  //NF N: 1 F: 0  0011
                              3 : send_buffer = 8'h0f;    //display on
                              4 : send_buffer = 8'h01; // display clear 
                              5 : send_buffer = 8'h06;
                         endcase
                        rs =0;  //�ʱ�ȭ �ڵ带 �Է��ϴ°��̱⿡ rs =0�� ���ش�.
                        send =1;    //������
                        cnt_data = cnt_data +1;
                    end                 
                end
                
                
                SEND_DATA : begin   //idle���� btn0������ ����Ʈ ������.
                    if(busy) begin //busy���¿��� �Ѿ�Ƿ� busy�� üũ�ϴ� �ڵ带 �����Ѵ�.
                        next_state = IDLE;  // send�� 1�Ǿ busy�� 1�Ǹ� �������� �Ѿ�´�.
                        send = 0;
                        if(cnt_data >= 9) cnt_data =0;
                        else cnt_data =cnt_data +1;
                    end
                    else begin
                        send_buffer = "A" + cnt_data;  //�빮�� A�� �ƽ�Ű ������ ����.
                            //A�� 0���� �ٲٸ� ���ڵ��� ���
                        rs =1; //������ �������̹Ƿ� rs =1;
                        send =1;    //busy�� 1�ɰ��� 
                    end
                end               
                    
                    
     //SEND_DATA���� RS�� 0�ְ�(������ �����۾�) SEND_BUFFER�� shift�� �۾� �ڵ� ����
                    SEND_COMMAND : begin   //idle���� btn0������ ����Ʈ ������.
                    if(busy) begin //busy���¿��� �Ѿ�Ƿ� busy�� üũ�ϴ� �ڵ带 �����Ѵ�.
                        next_state = IDLE;  // send�� 1�Ǿ busy�� 1�Ǹ� �������� �Ѿ�´�.
                        send = 0;
                    end
                    else begin
                        send_buffer =  8'h18;  //�� ����Ʈ 
                          //cursor on display shift���� (1(DB5)/ 1000(DB4321))
                        rs =0;  //RS�� 0�̸� ���ų� �����͸� �����ϴ� �۾��� �����ϰ� �����°��� �ƴ� 
                        send =1;    
                    end
                end           
                
                
                
                    // ���̳ʽ�(-)�� �ҰŸ� IDLE �ʱⰪ�� cnt_string=5�� ���ϰ� ���ذ��� �ٲ��ش�.
                    SEND_STRING : begin    
                    if(busy) begin  
                          send =0;
                          if(cnt_string <= 0) begin
                              next_state = IDLE;
                              cnt_string =5;
                          end
                      end
                      else if(!send)begin 
                          case(cnt_string)  
                              5 : send_buffer = hello[39: 32];
                                              // [8*cnt_string-1: 8*(cnt_string-1);�̰ǵ� ������ ���Ἥ ����� �����  
                                              // �� ������ ���� cnt_string�� ���������� ��� ����
                              4 : send_buffer = hello[31: 24];  
                              3 : send_buffer = hello[23: 16];  
                              2 : send_buffer = hello[15: 8];   
                              1 : send_buffer = hello[7: 0]; 
                         endcase
                             rs =1;     //���� LCD�� �������ϰ� rs=1�Ѵ�.
                             send =1;    //������
                             cnt_string = cnt_string - 1;
                      end                 
                     end             

                //SEND_COMMAND�� case������ ������ �������¸� SEND_DATA�� ������ ����
               SEND_COMMAND2 : begin   
               if(busy) begin 
                   send = 0;
                   if(command2_cnt_data >= 3) begin
                       next_state = IDLE;  
                       command2_cnt_data =0;
                   end
               end
                else if(!send) begin
                    case(command2_cnt_data)
                        0 : begin
                            send_buffer =  8'hc0;
                            rs = 0;
                        end
                       
                        1 : begin
                            send_buffer =  8'h0e;  //0 . 1110 >> display :on / cursor :on /blink:off
                            rs =0;
                         
                        end
                        
                        2 : begin
                            send_buffer = h[7:0];
                            rs =1;
                        end
                        
                    endcase
                   /*  rs =0; */
                     send =1;    
                     command2_cnt_data = command2_cnt_data +1;
                 end
               end      
                                       
         endcase
      end
  end                                             
endmodule


//=======================================================================




//======================================================
module servo_motor(
    input clk,reset_p,
    input [2:0] btn,
    output servo_motor_pwm,
    output [3:0] com,
    output [7:0] seg_7);
    
        reg [31:0] clk_div;     //�ӵ�����\
    always @(posedge clk or posedge reset_p) begin
        if(reset_p)clk_div = 20;
        else clk_div = clk_div +1;
    end

    wire clk_div_26_nedge;    //p edge   //������ 1 �ѹ��� ������ 0 >> T�ø��÷� ���                                          
    edge_detector_n ed(.clk(clk), .reset_p(reset_p),
                                                      .cp(clk_div[23]), .n_edge(clk_div_26_nedge));    
                                                      
    reg [6:0] duty;       // duty ���������� ũ�⸦ 8��Ʈ�� ����
    reg up_down;        // ���� ��� ���� �÷���
    reg duty_min, duty_max;
 
 wire btn_0, btn_1, btn_2;
      button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_0));
      button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_1));
      button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_2));
 
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duty = 12 ;       // �ʱ�ȭ 1ms (5% ��Ƽ ����Ŭ)
            up_down = 1;  // �ʱ� ���� ���� (0: ����, 1: ����)
            duty_min =12;
            duty_max = 50;
        end
        else if (clk_div_26_nedge) begin // 20ms �ֱ�
            if (up_down) begin
                if (duty <50)  // 2ms (10%)�� �������� �ʾҴٸ� ����
                    duty = duty + 1;
                else
                    up_down = 0;  // 2ms�� �����ϸ� ������ ���ҷ� ����
            end
            else begin
                if (duty > 12)  // 1ms (5%)�� �������� �ʾҴٸ� ����
                    duty = duty - 1;
                else
                    up_down = 1;  // 1ms�� �����ϸ� ������ ������ ����
            end
        end
            else if(btn_0)up_down = ~up_down;
            else if(btn_1)duty_min = duty;
            else if(btn_2)duty_max = duty;
     end   

    pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
    pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(servo_motor_pwm));
                                                                        
    wire [15:0] duty_bcd;  
    bin_to_dec bcd_humi(.bin({6'b0, duty}),  .bcd(duty_bcd));
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(duty_bcd),
                                                                        .com(com), .seg_7(seg_7)); 
endmodule


//==========================================================================
module pwm_Nstep_freq     //����ȭ
#( parameter sys_clk_freq = 100_000_000,    
    parameter pwm_freq = 10_000,                //led�� 10000hz���� ����
    parameter duty_step = 100,
    parameter temp = (sys_clk_freq/pwm_freq )/ duty_step,
    parameter temp_half = temp/2)

   (input clk, reset_p,
    input [31:0] duty, 
    output pwm);

    
     integer cnt_sysclk; 
     integer count_duty;          
     wire clk_freqxstep;
     wire clk_freqx100_nedge;
     
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_sysclk =0;
            else  begin
               if(cnt_sysclk >= temp-1) cnt_sysclk = 0; 
              else cnt_sysclk = cnt_sysclk +1;
            end  
       end   
                assign clk_freqxstep = (cnt_sysclk < temp_half) ? 1 : 0;    
            
     edge_detector_n ed_n1(.clk(clk), .reset_p(reset_p),
                                                   .cp(clk_freqxstep), .p_edge(clk_freqx100_nedge));


       always @(negedge clk or posedge reset_p)begin
            if(reset_p) count_duty =0;
            else if(clk_freqx100_nedge) begin
                if(count_duty >=(duty_step-1)) count_duty =0;
                else  count_duty = count_duty +1;    
            end  
       end   
                assign pwm = (count_duty <= duty) ? 1 : 0;  
                                                     //���� ���� ���� duty�� 0�϶��� 1�� ��µȴ�,  
endmodule


//===========================================================
//�����ļ��� slack ���
//�ý��ۼ����Ҷ� ������ �ϸ� ����
//===========================================================
module ultrasonic_sensor_cntr(
        input clk,reset_p,
        input echo1,
        output reg trig,
        output reg [50:0] distance,
        output [7:0] led_debug);
 

        
        parameter S_IDLE     = 4'b0001;
        parameter TRIGGER = 4'b0010;
        parameter ECHO_H   = 4'b0100;
        parameter ECHO_L   = 4'b1000;


        
        reg [3:0] state, next_state;

        //ms����
      wire clk_usec;
    clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p),
                                                               .clk_div_100_nedge(clk_usec));
      
      
      reg count_en;
      wire [11:0] cm;                                                         
     clock_div_58 div58(.clk(clk), .reset_p(reset_p), .clk_usec(clk_usec),
                                           .count_en(count_en), .cm(cm));                                                          
                                                               
     wire echo_p_edge, echo_n_edge;
    edge_detector_n ed13(.clk(clk), .reset_p(reset_p), .cp(echo1), 
                                                  .p_edge(echo_p_edge), .n_edge(echo_n_edge));   
      
         reg[21:0] count_usec;  //3�� ī��Ʈ�ϱ� ���� 22��Ʈ
         reg count_usec_en;   //enable
         
         reg[21:0] echo_time;   //������ ó���� ���� �ð��� ������ ����
         
          always @(posedge clk or posedge reset_p)begin
             if(reset_p) count_usec =0;
               else if(clk_usec && count_usec_en) count_usec = count_usec +1;
               else if(!count_usec_en) count_usec = 0;               
          end       
       
       //S_IDLE 
       //n���� state�ٲٰ�
       always @(posedge clk or posedge reset_p)begin
           if(reset_p) state = S_IDLE;
           else state = next_state;
       end       
       
          
    always @(negedge clk or posedge reset_p)begin
          if(reset_p)begin
              next_state = S_IDLE;
              count_usec_en = 0;
              trig = 0;
              echo_time=0;
              count_en =0;
            end
            else begin
            case(state)
   
            S_IDLE : begin
                if(count_usec < 22'd1_000_000)  begin   //1��
                   count_usec_en = 1;
                   
                end
                   else begin
                     count_usec_en = 0;
                     next_state = TRIGGER;
                     end    
            end
            
            
            TRIGGER : begin
                if(count_usec < 22'd10) begin
                    count_usec_en = 1;
                    trig =1;
                end
                else begin   
                    trig = 0;
                    count_usec_en =0;
                    next_state = ECHO_H;
                end         
            end
            
            
            ECHO_H : begin
                if(echo_p_edge) begin
                    count_usec_en = 1;
                    next_state = ECHO_L;
                    count_en =1;
                end
            end
            
                
            ECHO_L : begin
                if(echo_n_edge) begin
                    count_en =0;
                    distance = cm;      //distance ���� value���� �־����ϱ� cm
                   /* echo_time =count_usec; */
                   /* distance = count_usec / 58;   */         
                    next_state = S_IDLE;
                end            
                      else next_state = ECHO_L;
                end
            endcase
        end
     end   
                assign led_debug[3:0] =state;
     //������ ����(�Խ��� ����) 
     always @(posedge clk or posedge reset_p)begin
        if(reset_p)distance =0;
        else begin  //58�� ���
      
        end
     end
endmodule

//===============================================================
//58���� ����Ʈ���Ÿ� ������� �ʰ� �������
//===============================================================
module clock_div_58(
       input clk, reset_p,
       input clk_usec, count_en,      
       output reg[11:0] cm);
       
       reg  [5:0] count;       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) begin       //�ʱⰪ ����
            count = 0;
            cm = 0;
          end 
          else  if(clk_usec) begin        
                   if(count_en) begin
                   if(count >= 57) begin
                        count = 0;
                        cm = cm +1;
                    end 
                        else count = count +1;
                   end 
                
                end
                else if(!count_en) begin
                count = 0;
                cm =0;
               end
           end  
endmodule

//===========================================================

module keypad_cntr_FSM(
    input clk,reset_p,
    input [3:0] row,        //��
    output reg [3:0] col,      //��
    output reg [3:0] key_value,
    output reg key_valid);     
    
    parameter SCAN0               = 5'b00001;
    parameter SCAN1               = 5'b00010;
    parameter SCAN2               = 5'b00100;
    parameter SCAN3               = 5'b01000;
    parameter KEY_PROCESS = 5'b10000;
    

        reg[19:0] clk_div;
    always @(posedge clk)clk_div = clk_div +1;      //����ȸ��
  
    wire clk_8msec_n, clk_8msec_p;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(clk_div[19]), 
                                                  .p_edge(clk_8msec_p), .n_edge(clk_8msec_n)); 
      
      reg[4:0] state, next_state;

        always@(posedge clk or posedge reset_p)begin      
            if(reset_p)state =SCAN0;
            else if(clk_8msec_p)state =next_state;
        end

        always@(*)begin     //����ȸ��    FSM
            case(state) 
                SCAN0 : begin
                        if(row ==0) next_state =  SCAN1;    //Ű�Է��� ������ Ű ���μ���
                        else next_state = KEY_PROCESS;          //next_state���� = ���� ��������(�ø��÷�)���� ���ȴ�.
                        end
                SCAN1 : begin
                        if(row ==0) next_state = SCAN2;
                        else next_state = KEY_PROCESS;
                        end
                SCAN2 : begin
                        if(row ==0) next_state = SCAN3;
                        else next_state = KEY_PROCESS;          
                        end
                SCAN3 : begin
                        if(row ==0) next_state = SCAN0;
                        else next_state = KEY_PROCESS;
                  end       
                 KEY_PROCESS : begin
                        if(row ==0) next_state = SCAN0;
                        else next_state = KEY_PROCESS;                 
                     end
                        
                  default : next_state = SCAN0;                                 
            endcase
        end
        
        always@(posedge clk or posedge reset_p) begin
            if(reset_p) begin
                key_value = 0;
                key_valid = 0;
                col =0;
              end
              else if(clk_8msec_n)begin
                    case(state)     //������ ���� ����
                        SCAN0 : begin 
                                 col= 4'b0001; 
                                 key_valid = 0; 
                           end
                        SCAN1 : begin
                                 col= 4'b0010; 
                                 key_valid = 0; 
                            end
                        SCAN2 : begin
                                 col= 4'b0100; 
                                 key_valid = 0; 
                            end
                        SCAN3 : begin
                                 col= 4'b1000; 
                                 key_valid = 0; 
                            end
                            
                        KEY_PROCESS : begin
                                key_valid = 1;
                                case({col, row})    
                                    8'b0001_0001 : key_value = 4'h0;        //���� �ٲٰ� �ٷ� ����
                                    8'b0001_0010 : key_value = 4'h4;
                                    8'b0001_0100 : key_value = 4'h8;
                                    8'b0001_1000 : key_value = 4'hc;
                                    8'b0010_0001 : key_value = 4'h1;
                                    8'b0010_0010 : key_value = 4'h5;
                                    8'b0010_0100 : key_value = 4'h9;
                                    8'b0010_1000 : key_value = 4'hd;
                                    8'b0100_0001 : key_value = 4'h2;
                                    8'b0100_0010 : key_value = 4'h6;
                                    8'b0100_0100 : key_value = 4'ha;
                                    8'b0100_1000 : key_value = 4'he;
                                    8'b1000_0001 : key_value = 4'h3;
                                    8'b1000_0010 : key_value = 4'h7;
                                    8'b1000_0100 : key_value = 4'hb;
                                    8'b1000_1000 : key_value = 4'hf;        
                                    endcase             
                        end
                    endcase
              end
        end
endmodule


//==============================================================
module down_counter60(
    input clk,reset_p,
    input out_data);
    


endmodule

//======================================================================
module timer_1m(
    input clk, reset_p,
    output [3:0] com,
    output [7:0] seg_7);
    
    wire btn_pedge;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn), .btn_pedge(btn_pedge));
   
    wire clk_usec, clk_msec, clk_sec;
    clock_div_100 clk_us(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 clk_ms( .clk(clk) , .reset_p(reset_p) , 
                                      .clk_source(clk_usec) , .clk_div_1000(clk_msec));
    clock_div_1000 clk_s( .clk(clk) , .reset_p(reset_p) , 
                                    .clk_source(clk_msec) , .clk_div_1000_nedge(clk_sec));  

    wire clk_sec_n;
   edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p),   
                                          .cp(clk_sec),  .n_edge(clk_sec_n));
    
    reg [5:0] count;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) count=60; 
        else if(clk_sec_n) begin
            if(count == 0) count = 0;
           else count = count -1;
       end
     end
    
    
    //count�� timer�� ��Ʈ���� �ǵ帮�������� fnd ������ 2���� ���� �����ְ������
    wire [15:0] timer_value;
    bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                      .com(com), .seg_7(seg_7));

endmodule
