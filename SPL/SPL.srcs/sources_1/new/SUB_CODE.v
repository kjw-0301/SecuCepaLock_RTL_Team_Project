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
module fnd_cntr(     //Ïª®Ìä∏Î°§Îü¨  //fnd 0?ùº?ïå ÏºúÏßÑ?ã§.  
        input clk, reset_p,
        input [15:0] value,
        output [3:0] com,       //Í≥µÌÜµ?ã®?ûê  //LED?ùò ?†Ñ?õê ?ã¥?ãπ
        output [7:0] seg_7);
        
        ring_counter_fnd rc(clk, reset_p, com);
         // Î™®ÎìàÎ™?         ?ù∏?ä§?Ñ¥?ä§Î™?
        reg [3:0] hex_value;     //decoder_7seg?óê ?ì§?ñ¥Í∞? reg
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
//Î≤ÑÌäº Ïª®Ìä∏Î°§Îü¨
//=============================================================
module button_cntr(
        input clk, reset_p,
        input btn,
        output btn_pedge, btn_nedge);
        
         reg[20:0] clk_div = 0; 
         always @(posedge clk)clk_div = clk_div +1;
       
       
         //Ï±ÑÌÑ∞Îß? Í≥ºÏ†ï ?ä§???ä∏
         wire clk_div_nedge;  
    edge_detector_n ed(.clk(clk), .reset_p(reset_p),
                                               .cp(clk_div[16]), .n_edge(clk_div_nedge)); 
                      
                      
         //Ï±ÑÌÑ∞Îß? ?û°?äî Í≥ºÏ†ï                         
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
// T?îåÎ¶ΩÌîåÎ°? (?ÉÅ?äπ?ó£Ïß?)
/////////////////////////////////////////////////////////////////////////////////////////////////////////
module T_flip_flop_positive(
    input clk, reset_p,
    input t,
    output reg q
);
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) q = 0;
        else begin
            if (t) q = ~q; // tÍ∞? 1?ù¥Î©? q?äî Î∞òÏ†Ñ
            else q = q; // ?Éù?ûµ Í∞??ä•
        end 
    end
     
endmodule
//=======================================================================
module clock_div_100(
       input clk, reset_p,
       output clk_div_100,     
       output clk_div_100_nedge);
       
       reg  [6:0] cnt_sysclk;       //sysclk :?ãú?ä§?Öú ?Å¥?ùΩ   10-6
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_sysclk =0;
          
          else  begin
              if(cnt_sysclk >= 99) cnt_sysclk = 0; 
             else cnt_sysclk = cnt_sysclk +1;
            
            end  
       end   
                assign clk_div_100 = (cnt_sysclk < 50) ? 0 : 1;     
                
    edge_detector_n ed1(        //?ã§?ö¥ ?ó£Ïß? ?îî?Öç?Ñ∞?óê 100Î∂ÑÏ£º ?Å¥?ü≠+?ã§?ö¥?ó£Ïß?    
                                                         //?†Ñ?óê ÏΩîÎìú?óê?Ñú Î∂àÎü¨?ò§Í∏?
        .clk(clk),
        .reset_p(reset_p),    //(?ó£Ïß??äî ck?Åù?ÇòÍ≥? 1?Å¥?ü≠?í§?óê ?Çò?ò®?ã§)
        .cp(clk_div_100),  
        .n_edge(clk_div_100_nedge));
      
    
endmodule

//=======================================================================

module clock_div_60(
       input clk, reset_p,
       input clk_source,            //1us clock
       output clk_div_60,     //100Î∂ÑÏ£º(Í∏?)
       output clk_div_60_nedge);
       
       reg  [9:0] cnt_clksource;       //sysclk :?ãú?ä§?Öú ?Å¥?ùΩ       10-9
     //Î∂?Ï°±ÌïòÎ©? ?äò?†§?ïº?ïòÏß?Îß? ?ÑòÏπòÎ©¥ Ï§ÑÏù¥?äîÍ±? ?Ñ†?Éù?ù¥?ã§.
     // Í≥†Ïπ†Í∫ºÎ©¥ 5Î°? Î∞îÍæ∏Î©? ?êú?ã§.
     // Ï£ºÎ°ú integer cnt_clksourceÎ°? ?ì∞Í∏∞ÎèÑ?ïú?ã§ (32ÎπÑÌä∏)
     // 32ÎπÑÌä∏Î•? ?Ñò?ñ¥Í∞?Î©? ???†áÍ≤? ?ì∞Í∏∞ÎèÑ ?Ñ§?†ï?ï¥Ï§òÏïº?ïú?ã§.
     
     
       wire clk_source_nedge;         //?óêÏß? ?îî?Öç?Ñ∞ ?ïòÍ∞ïÏó£Ïß? Î∂àÎü¨?ò§Í∏?
               edge_detector_n ed1(.clk(clk), 
                                                        .reset_p(reset_p),    
                                                        .cp(clk_source),  
                                                        .n_edge(clk_source_nedge));
       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_clksource =0;       //Ï¥àÍ∏∞Í∞? ?Ñ∏?åÖ
          
          else  if(clk_source_nedge) begin        //clk_sourceÍ∞? 1?ì§?ñ¥?ò¨?ïåÎßàÎã§
                   if(cnt_clksource >= 59) cnt_clksource = 0; 
               else cnt_clksource = cnt_clksource +1;         //1us?óê 1?î©Ï¶ùÍ??ï¥?Ñú 1msÎ°?
            
            end  
       end   
                assign clk_div_60 = (cnt_clksource < 30) ? 0 : 1;     
                
    edge_detector_n ed2(              //?ã§?ö¥ ?ó£Ïß? ?îî?Öç?Ñ∞?óê 100Î∂ÑÏ£º ?Å¥?ü≠+?ã§?ö¥?ó£Ïß?
        .clk(clk), .reset_p(reset_p),    //(?ó£Ïß??äî ck?Åù?ÇòÍ≥? 1?Å¥?ü≠?í§?óê ?Çò?ò®?ã§)
        .cp(clk_div_60),  .n_edge(clk_div_60_nedge));
    
endmodule

//==============================================================================

module clock_div_1000(
       input clk, reset_p,
       input clk_source,            //1us clock
       output clk_div_1000,     //100Î∂ÑÏ£º(Í∏?)
       output clk_div_1000_nedge);
       
       reg  [9:0] cnt_clksource;       //sysclk :?ãú?ä§?Öú ?Å¥?ùΩ       10-9
     
       wire clk_source_nedge;         //?óêÏß? ?îî?Öç?Ñ∞ ?ïòÍ∞ïÏó£Ïß? Î∂àÎü¨?ò§Í∏?
               edge_detector_n ed1(.clk(clk), 
                                                        .reset_p(reset_p),    
                                                        .cp(clk_source),  
                                                        .n_edge(clk_source_nedge));
       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_clksource =0;       //Ï¥àÍ∏∞Í∞? ?Ñ∏?åÖ
          
          else  if(clk_source_nedge) begin        //clk_source_nedgeÍ∞? 1?ì§?ñ¥?ò¨?ïåÎßàÎã§
                   if(cnt_clksource >= 999) cnt_clksource = 0; 
               else cnt_clksource = cnt_clksource +1;         //1us?óê 1?î©Ï¶ùÍ??ï¥?Ñú 1msÎ°?
            
            end  
       end   
                assign clk_div_1000 = (cnt_clksource < 500) ? 0 : 1;     
                
    edge_detector_n ed2(              //?ã§?ö¥ ?ó£Ïß? ?îî?Öç?Ñ∞?óê 100Î∂ÑÏ£º ?Å¥?ü≠+?ã§?ö¥?ó£Ïß?
        .clk(clk), .reset_p(reset_p),    //(?ó£Ïß??äî ck?Åù?ÇòÍ≥? 1?Å¥?ü≠?í§?óê ?Çò?ò®?ã§)
        .cp(clk_div_1000),  .n_edge(clk_div_1000_nedge));
    
endmodule
//==============================================================
//Î°úÎìúÍ∞? Í∞??ä•?ïú 60Ïß? Ïπ¥Ïö¥?Ñ∞ (156~160)
//==============================================================
module loadable_counter_bcd_60(
      input clk,reset_p,
      input clk_time,
      input load_enable,
      input [3:0] load_bcd1, load_bcd10,
      output reg [3:0] bcd1, bcd10); //1?ùò ?ûêÎ¶? , 10?ùò ?ûêÎ¶?
      
      wire clk_time_nedge;
          edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p),   
                                     .cp(clk_time),  .n_edge(clk_time_nedge));
      
      always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin       //Î¶¨ÏÖã Í∞? 
            bcd1 = 0;
            bcd10= 0;
        end
        else begin
                            //Î≥?Í≤ΩÏ†ê
                            //bcd1Í≥? 10?ùò Î°úÎìú?êúÍ∞íÏùÑ ?ã§?ãú ?Ñ£?äî?ã§.
                    if(load_enable)begin
                        bcd1 = load_bcd1;
                        bcd10 = load_bcd10;
                    end   
                        else if(clk_time_nedge) begin
                            if(bcd1 >= 9) begin
                                bcd1 =0;
                          if(bcd10 >= 5) bcd10 = 0;       //60Î∂ÑÏ£º?ãàÍπ? 60?ù¥Î©? 0?úºÎ°? Í∞??ûê
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
    input rd_wr,        //Ï£ºÏÜå ?ùΩÍ±∞ÎÇò ?ì∞Í±∞ÎÇò
    input [7:0] data,
    input comm_go,  //?ù¥Í≤? 0->1?ì§?ñ¥?ò§Î©? ?Üµ?ã†?ãú?ûë
    output reg  scl, sda,
    output reg [15:0] led_debug);
     

    
    
    parameter IDLE =  7'b000_0001;
    parameter  COMM_START=  7'b000_0010;
    parameter  SEND_ADDR=  7'b000_0100;
    parameter  RD_ACK=  7'b000_1000;
    parameter  SEND_DATA=  7'b001_0000;
    parameter  SCL_STOP=  7'b010_0000;      //?Å¥?ùΩ ?ä§?Éë (?Üç?èÑ 100KHZ?†ï?èÑ) 10us
    parameter  COMM_STOP=  7'b100_0000;


   //CLOCK US       
   wire clk_usec;
    //usÏ¥?
   clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));     

   //EDGE_DETECTOR  
   wire scl_pedge, scl_nedge, comm_go_pedge;
 
   edge_detector_n SCL_P_N(.clk(clk), .reset_p(reset_p), .cp(scl),  
                                                   .p_edge(scl_pedge), .n_edge(scl_nedge));
  
   edge_detector_n COMM_GO_P(.clk(clk), .reset_p(reset_p), .cp(comm_go),  
                                                         .p_edge(comm_go_pedge));


    //Î∏îÎ°úÍ∑? ADDRESS PACKET  FORMAT Ï∞∏Ï°∞
    //10us?Ç¨?ö© ?ïò?äî?ç∞ en?ã†?ò∏Î•? Ï∂îÍ??ïú Í∞úÎÖê?ù¥?ã§.(SCL?†ú?ñ¥)
    reg [2:0] count_usec5;
    reg scl_e;  //clock_enable 
    always @(posedge clk or posedge reset_p) begin
        if(reset_p)  begin
            count_usec5 = 0; 
            scl =1;         //scl?? Ï≤òÏùå?óê 1
        end
        
        else if(scl_e)begin         //scl_e?äî scl ?ù¥ 1?ùº¬ã¬öÎß? ?èô?ûë 
            if(clk_usec) begin
                if(count_usec5 >= 4)begin
                    count_usec5 =0;
                    scl = ~scl;
                end
                else  count_usec5 = count_usec5 +1; 
            end
        end   
        
            else if(!scl_e) begin    //scl_e ?ù¥ 0?ù¥?ùºÎ©?
                scl =1;                 //scl?? ?Åù?Ç†¬ã¬ö 1Î°? ?Åù?Çú?ã§.
                count_usec5 = 0;
            end   
    end                                               
   
   
   //state, Ï£ºÏÜå ?Ñ†?ñ∏
    reg [6:0] state ,next_state;     
    wire [7:0] addr_rw;
    assign addr_rw = {addr, rd_wr}; //Ï£ºÏÜå, Read/Write ?ï©Ï≥êÏÑú 8ÎπÑÌä∏ 
    
    reg [2:0] cnt_bit;
    reg stop_flag;  //?ç∞?ù¥?Ñ∞ Î≥¥ÎÇ¥Í≥? ?ÇòÎ©? 1Î°?(RD_ACK?óê?Ñú ?Ç¨?ö©)
    
    //MAIN CODE(?îÑÎ°úÌÜ†ÏΩ? Íµ¨ÌòÑ)
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state = IDLE;
        else  state = next_state;
    end
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state =IDLE;
            scl_e =0;   //clk ?? 0?úºÎ°?
            sda =1;
            cnt_bit =7; //ÏµúÏÉÅ?úÑ ÎπÑÌä∏ Î≥¥ÎÇºÍ∫ºÎãàÍπ? 7Î°? Ï¥àÍ∏∞?ôî
            stop_flag =0; 
            led_debug = 0;  
        end
        else begin
            case(state)
                IDLE : begin
                led_debug[0] = 1;
                    scl_e =0;
                    sda =1;
                    if(comm_go_pedge) next_state = COMM_START;   //comm_goÍ∞? 1?ì§?ñ¥?ò§Î©? ?Ñò?ñ¥Í∞?
                end
                
                COMM_START : begin  //SCL?? 1?ù∏?ÉÅ?Éú?óê?Ñú SDAÍ∞? 0?úºÎ°? ?ñ®?ñ¥Ïß?, ?Üµ?ã†?ãú?ûë>>clock??ÏßÅÏûÑ
                    led_debug[1] = 1;
                    sda =0;
                    scl_e =1;   //clock ??ÏßÅÏûÑ >>5us?õÑ?óê 0?ù¥ ?êú?ã§.
                    next_state = SEND_ADDR;
                end 
                
                SEND_ADDR : begin
                led_debug[2] = 1;
                    if(scl_nedge) sda =addr_rw[cnt_bit]; //Ï≤òÏùå?óê 7Î≤àÎπÑ?ä∏ Ï§??ã§.
                    else if(scl_pedge) begin
                        if(cnt_bit ==0) begin 
                            cnt_bit =7;
                            next_state =RD_ACK;  //0?ù¥ ?êòÎ©? next_stateÎ°?
                        end
                        else cnt_bit = cnt_bit -1; //p_edge?óê?Ñ† ?ïÑÎ¨¥Í≤É?èÑ ?ïà?ïò?ãàÍπ? Í∑∏Îã§?ùåÎπÑÌä∏?óê?Ñú Î∫ÑÍ∫º ÎØ∏Î¶¨ Ï§?Îπ?
                    end                                          //Ï¶? 7Î∂??Ñ∞ 0ÍπåÏ?  Ï≠? ÎπºÎäî Î∞òÎ≥µÎ¨? ?Ñ§?†ï?ïúÍ±∞ÏûÑ
                end 
                
                RD_ACK : begin  //RD_ACK?ã§?ùå?? SEND_DATAÎ°? Í∞àÏ? SCL_STOP?úºÎ°? Í∞àÏ? ?†ï?ï¥Ï§òÏïº?ï®(2Í∞?Ïß? Í≤ΩÏö∞)
                    led_debug[3] = 1;
                    if(scl_nedge) sda ='bz; 
                    else if(scl_pedge) begin        //pedge?ì§?ñ¥?ò§Î©? 1?Å¥?ùΩ ?Åù?ÇúÍ±∞ÏûÑ, clk?? Î≥¥ÎÇ¥?ïº?ïò?ãàÍπ? ??
                        if(stop_flag) begin //Ï¥àÍ∏∞?ôî Í∞íÏ? 0
                            stop_flag =0;   //SCL ?ÅùÍ≥ºÏ†ï
                            next_state =SCL_STOP;
                        end
                        else begin
                            stop_flag =1;   // SCL Ï§ëÍ∞ÑÍ≥ºÏ†ï 
                            next_state = SEND_DATA;
                        end
                    end
                end 
                
                SEND_DATA : begin
                led_debug[4] = 1;
                if(scl_nedge) sda =data[cnt_bit]; //Ï≤òÏùå?óê 7Î≤àÎπÑ?ä∏ Ï§??ã§.
                    if(scl_pedge) begin
                        if(cnt_bit == 0) begin 
                            cnt_bit =7;
                            next_state =RD_ACK; //0?ù¥ ?êòÎ©? next_stateÎ°?
                        end
                        else cnt_bit = cnt_bit -1; //p_edge?óê?Ñ† ?ïÑÎ¨¥Í≤É?èÑ ?ïà?ïò?ãàÍπ? Í∑∏Îã§?ùåÎπÑÌä∏?óê?Ñú Î∫ÑÍ∫º ÎØ∏Î¶¨ Ï§?Îπ?
                    end         
                end 
                
                SCL_STOP : begin
                 led_debug[5] = 1;
                    if(scl_nedge) sda =0;
                    else if(scl_pedge) next_state = COMM_STOP;
                end 
                
                COMM_STOP : begin   //high ?ÉÅ?Éú?óê?Ñú Ï°∞Í∏à?ûà?î∞Í∞? Ï§??ã§.
                    led_debug[6] = 1;
                    if(count_usec5 >= 3)begin//3usÎß? Í∏∞Îã§Î¶¨Îã§Í∞?
                        scl_e =0;   //  ?Å¥?ùΩ?ù¥ highÎ°? ?ú†Ïß??êòÎ©? 
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
    input rs, send,     //sendÍ∞? 1?ùº¬ã¬ö comm_go?óê 1¬ã¬ö?†§?Ñú ?ÉÅ?úÑ 4ÎπÑÌä∏ Î≥¥ÎÇ¥Í≥?/ enable ?ïò?úÑ 4ÎπÑÌä∏Î≥¥ÎÇ¥Í≥? /enable
    input [7:0] send_buffer,    //data?ûÑ
    output scl, sda,
    output reg busy  //busyÍ∞? 0?ùº?ïåÎß? sendÎ•? Î≥¥ÎÇº?àò?ûàÍ≤? (?ãúÍ∞ÑÏù¥ Ï¢? Í±∏Î¶¨?ãàÍπ?)
);


    //PARAMETER
                    //NIBBLE= 4ÎπÑÌä∏
    reg [5:0] state, next_state;
    parameter IDLE                                        = 6'b00_0001; 
    parameter SEND_HIGH_NIBBLE_DISABLE = 6'b00_0010;//?ÉÅ?úÑ 4ÎπÑÌä∏Î•? Î≥¥ÎÇºÍ±¥Îç∞ disableÎ°? ?ïúÎ≤àÎ≥¥?Ç¥Í≥?
    parameter SEND_HIGH_NIBBLE_ENABLE  = 6'b00_0100; //?ÉÅ?úÑ 4ÎπÑÌä∏Î•? Î≥¥ÎÇºÍ±¥Îç∞ enableÎ°? ?ïúÎ≤àÎ≥¥?Ç¥Í≥?
    parameter SEND_LOW_NIBBLE_DISABLE  = 6'b00_1000;//?ïò?úÑ 4ÎπÑÌä∏Î•? Î≥¥ÎÇºÍ±¥Îç∞ disableÎ°? ?ïúÎ≤àÎ≥¥?Ç¥Í≥?
    parameter SEND_LOW_NIBBLE_ENABLE   = 6'b01_0000;//?ïò?úÑ 4ÎπÑÌä∏Î•? Î≥¥ÎÇºÍ±¥Îç∞ enableÎ°? ?ïúÎ≤àÎ≥¥?Ç¥Í≥?
    parameter SEND_DISABLE                        = 6'b10_0000;//disable?ã†?ò∏Î•? Î≥¥ÎÇ¥?Ñú Í∞íÏùÑ Î™ªÏùΩÍ≤? ?ïú?ã§.
                          // LCDÍ∏∞Ï? 1Î∞îÏù¥?ä∏(8ÎπÑÌä∏) Î∞õÎäîÍ±∞ÏûÑ 
    
    reg [7:0] data;
    reg comm_go; //?ù¥Í≤? 0->1?ì§?ñ¥?ò§Î©? ?Üµ?ã†?ãú?ûë
    I2C_controller(.clk(clk) , .reset_p(reset_p) ,
                           .addr(addr) , .rd_wr(0) , // .rd_wr(0) : ?ì∞Í∏∞Îßå
                           .data(data), .comm_go(comm_go), 
                           .scl(scl), .sda(sda), .led_debug(led_debug));
    
   //CLOCK US(usÏ¥?)
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
                    if(count_usec <= 22'd200)  begin            //200us //start, I2CÍ≥ºÏ†ï(18)  , stop ?ï¥?Ñú ?ó¨?ú†Î°?Í≤? 200us
                        data= {send_buffer[7:4], 3'b100, rs};   // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4ÍπåÏ?)
                        comm_go =1;                                     //BTÎß? 1?ù∏Í±∞ÏûÑ
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_HIGH_NIBBLE_ENABLE;   
                    end
                 end 

                 SEND_HIGH_NIBBLE_ENABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2CÍ≥ºÏ†ï(18)  , stop ?ï¥?Ñú ?ó¨?ú†Î°?Í≤? 200us
                        data= {send_buffer[7:4], 3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4ÍπåÏ?)
                        comm_go =1;                  //BTÎß? 1?ù∏Í±∞ÏûÑ
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_LOW_NIBBLE_DISABLE;   
                    end
                  end 
                 
                 SEND_LOW_NIBBLE_DISABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2CÍ≥ºÏ†ï(18)  , stop ?ï¥?Ñú ?ó¨?ú†Î°?Í≤? 200us
                        data= {send_buffer[3:0], 3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4ÍπåÏ?)
                        comm_go =1;                  //BTÎß? 1?ù∏Í±∞ÏûÑ
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_LOW_NIBBLE_ENABLE;   
                    end
                 end                      
                 
                 SEND_LOW_NIBBLE_ENABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2CÍ≥ºÏ†ï(18)  , stop ?ï¥?Ñú ?ó¨?ú†Î°?Í≤? 200us
                        data= {send_buffer[3:0], 3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4ÍπåÏ?)
                        comm_go =1;                  //BTÎß? 1?ù∏Í±∞ÏûÑ
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_DISABLE;   
                        end
                 end
                 
                 SEND_DISABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2CÍ≥ºÏ†ï(18)  , stop ?ï¥?Ñú ?ó¨?ú†Î°?Í≤? 200us
                        data= {send_buffer[3:0], 3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4ÍπåÏ?)
                        comm_go =1;                  //BTÎß? 1?ù∏Í±∞ÏûÑ
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = IDLE;   
                        busy =0;    //IDLEÎ°? Í∞à¬ã¬? busyÍ∞? 1 , ?ÇòÎ®∏Ï??óê?Ñ† 0
                     end               //?ã§ Î≥¥ÎÇ¥Í≥? ?ÇòÎ©? busyÍ∞? 0?ù¥?ùºÍ≥? ?ÉùÍ∞? 
                 end
            endcase
          end
      end
endmodule

/////////////////////////////////////////////////////////////////////////////

module servo_motor(
    input clk,reset_p,
    input open,
    input btn_close,
    input close,    
    output servo_motor_pwm);


    reg [30:0] clk_div;
    wire clk_div_n;
    always @(posedge clk) clk_div = clk_div +1;
    edge_detector_n n(.clk(clk), .reset_p(reset_p), .cp(clk_div[23]), .n_edge(clk_div_n));


    reg[6:0] duty;
    reg up_down;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
        duty =12;
        end

        else if(open) begin //open
            duty =12;
        end

        else if(close | btn_close) begin    //close
            duty =27;
        end
      end

    pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
    pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(servo_motor_pwm));

    wire [15:0] duty_bcd;
 

endmodule

module servo_motor_test(
    input clk,
    input reset_p,
    input open_close,    // open∞˙ close∏¶ ≈Î«’«— Ω≈»£
    output servo_motor_pwm,
    output btn_close
);

    reg [30:0] clk_div;
    wire clk_div_n;

    always @(posedge clk) clk_div = clk_div + 1;
    edge_detector_n n(.clk(clk), .reset_p(reset_p), .cp(clk_div[23]), .n_edge(clk_div_n));

    reg [6:0] duty;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duty = 12; // √ ±‚ ªÛ≈¬¥¬ ¥›»˚
        end else if (open_close) begin
            duty = 12; // open ªÛ≈¬
        end else begin
            duty = 27; // close ªÛ≈¬
        end
    end

    pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
    pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(servo_motor_pwm));

endmodule



//==========================================================================
module pwm_Nstep_freq     //Í≥µÏãù?ôî
#( parameter sys_clk_freq = 100_000_000,    
    parameter pwm_freq = 10_000,                //led?äî 10000hz?óê?Ñú ?èô?ûë
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
                                                     //Í∞ôÎã§ ÍπåÏ? ?ÅºÎ©? dutyÍ∞? 0?ùº?ïå?èÑ 1?ù¥ Ï∂úÎ†•?êú?ã§,  
endmodule


//===========================================================
//Ï¥àÏùå?åå?Ñº?Ñú slack ?û°Í∏?
//?ãú?ä§?Öú?Ñ§Í≥ÑÌï†?ïê ?Çò?àÑÍ∏? ?ïòÎ©? Íº¨ÏûÑ
//===========================================================
module HC_SR04_cntr (
    input clk, reset_p, 
    input hc_sr04_echo,
    output reg hc_sr04_trig,
    output reg [21:0] distance);
    
    reg [3:0] state, next_state;
    // For Test
    
    // Define state 
    parameter S_IDLE              = 4'b0001;
    parameter S_10US_TTL          = 4'b0010;
    parameter S_WAIT_PEDGE        = 4'b0100;
    parameter S_CALC_DIST         = 4'b1000;
        
    // Define state, next_state value.
    
    
    // æ¡¶ next_state∏¶ state ∫Øºˆø° ≥÷¥¬∞°?
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) state = S_IDLE;
        else state = next_state;
    end
    
    // get 10us negative one cycle pulse
    wire clk_usec;
    clock_div_100   usec_clk( .clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));     // 1us
    wire [11:0] cm;
    sr04_div_58 sro(.clk(clk), .reset_p(reset_p), .clk_usec(clk_usec), .cnt_e(cnt_e), .cm(cm));
     
        
        
    reg [21:0] counter_usec;    
    reg counter_usec_en;
    reg cnt_e;
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin counter_usec = 0;
        end else if(clk_usec && counter_usec_en) counter_usec = counter_usec + 1;
        else if(!counter_usec_en) counter_usec = 0;
    end
    
    
    // hc_sr04_data¿« Negative edge, Positive edge æÚ±‚.
    wire hc_sr04_echo_n_edge, hc_sr04_echo_p_edge;
    edge_detector_n edge_detector_0 (.clk(clk), .reset_p(reset_p), .cp(hc_sr04_echo), .n_edge(hc_sr04_echo_n_edge), .p_edge(hc_sr04_echo_p_edge));
    
    // ªÛ≈¬ √µ¿Ãµµø° µ˚∏• caseπÆ ¡§¿«
    // ∞¢ ªÛ≈¬ø° µ˚∏• µø¿€ ¡§¿«
    
    reg[21:0] echo_time;
    
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = S_IDLE;
            counter_usec_en = 0;  
            echo_time =0;
            cnt_e =0;
        end else begin
            case(state)
                S_IDLE : begin        
                    if(counter_usec < 22'd3_000_000) begin
                        counter_usec_en = 1;  
                        hc_sr04_trig = 0;
                    end
                    else begin
                        counter_usec_en = 0;
                        next_state = S_10US_TTL;
                    end
                end
                
                
                
                S_10US_TTL : begin
                    if(counter_usec < 22'd10) begin
                        counter_usec_en = 1;
                        hc_sr04_trig = 1;
                    end
                    else begin
                        hc_sr04_trig = 0;
                        counter_usec_en = 0;
                        next_state = S_WAIT_PEDGE;
                    end
                end
                
                
                
                S_WAIT_PEDGE :  
                    if(hc_sr04_echo_p_edge) begin
                         next_state = S_CALC_DIST;    
                         cnt_e = 1;
                    end     
                
                
                
                S_CALC_DIST : begin          
                     if(hc_sr04_echo_n_edge) begin
                                distance = cm ;
                                cnt_e =0;
                                counter_usec_en = 0;
                                next_state = S_IDLE;
                      end
                      else next_state = S_CALC_DIST;
                end
            endcase
        end
    end
    
//    always @(posedge clk or posedge reset_p)begin
//        if(reset_p)distance = 0;
//        else begin
//           if(echo_time < 174)distance = 2;
//           else if(echo_time < 232)distance = 3;
//           else if(echo_time <290) distance = 4;
//           else if(echo_time <348) distance = 5;
//           else if(echo_time <406) distance = 6;
//           else if(echo_time <464) distance = 7;
//           else if(echo_time <522) distance = 8;
//           else if(echo_time <580) distance = 9;
//           else if(echo_time <638) distance = 10;
//           else if(echo_time <696) distance = 11;
//           else if(echo_time <754) distance = 12;
//           else if(echo_time <812) distance = 13;
//           else if(echo_time <870) distance = 14;
//           else if(echo_time <928) distance = 15;
//           else if(echo_time <986) distance = 16;
//           else if(echo_time < 1044) distance <= 17;
//            else if(echo_time < 1102) distance <= 18;
//            else if(echo_time < 1160) distance <= 19;
//            else if(echo_time < 1218) distance <= 20;
//            else if(echo_time < 1276) distance <= 21;
//            else if(echo_time < 1334) distance <= 22;
//            else if(echo_time < 1392) distance <= 23;
//            else if(echo_time < 1450) distance <= 24;
//            else if(echo_time < 1508) distance <= 25;
//            else if(echo_time < 1566) distance <= 26;
//            else if(echo_time < 1624) distance <= 27;
//            else if(echo_time < 1682) distance <= 28;
//            else if(echo_time < 1740) distance <= 29;
//            else if(echo_time < 1798) distance <= 30;
//            else if(echo_time < 1856) distance <= 31;
//            else if(echo_time < 1914) distance <= 32;
//            else if(echo_time < 1972) distance <= 33;
//            else if(echo_time < 2030) distance <= 34;
//            else if(echo_time < 2088) distance <= 35;
//            else if(echo_time < 2146) distance <= 36;
//            else if(echo_time < 2204) distance <= 37;
//            else if(echo_time < 2262) distance <= 38;
//            else distance <= 39; // for any echo_time larger than 2262, default to 39
 //       end
//    end
endmodule           

module sr04_div_58(
    input clk, reset_p,
    input clk_usec,cnt_e,
    output reg [11:0] cm);

 reg [5:0] cnt;
 
    always @(negedge clk or posedge reset_p)begin
         if(reset_p)begin
            cnt = 0;
            cm=0;
         end
         else if(clk_usec) begin
           if(cnt_e)begin
                if (cnt >= 57) begin
                cnt = 0;  //ƒ´øÓ≈Õ∞° 99∏¶ ºº∏È 0¿∏∑Œ √ ±‚»≠
                cm = cm +1;
             end
                else cnt = cnt + 1;      //1æø ¡ı∞° 
            end
          end
          else if(!cnt_e)begin
            cnt =0;
            cm = 0;
    end
 end       

endmodule













//===============================================================
//58Î∂ÑÏ£º ?ó£Ïß??ä∏Î¶¨Í±∞Î•? ?Ç¨?ö©?ïòÏß? ?ïäÍ≥? ?ì∞?äêÎ∞©Î≤ï
//===============================================================
module clock_div_58(
       input clk, reset_p,
       input clk_usec, count_en,      
       output reg[11:0] cm);
       
       reg  [5:0] count;       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) begin       //Ï¥àÍ∏∞Í∞? ?Ñ∏?åÖ
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
module key_pad_cntr_FSM(
        input clk, reset_p,
        input [3:0] row,
        output reg [3:0] col,
        output reg [3:0] key_value,
        output reg key_valid);
    
        parameter SCAN0          = 5'b00001; 
        parameter SCAN1          = 5'b00010; 
        parameter SCAN2          = 5'b00100; 
        parameter SCAN3          = 5'b01000; 
        parameter KEY_PROCESS    = 5'b10000; 
        
        reg [19:0] clk_div;
        
        always @(posedge clk) clk_div = clk_div +1;
        wire clk_8msec_p, clk_8msec_n;
        edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(clk_div[19]), .p_edge(clk_8msec_p), .n_edge(clk_8msec_n));
    
    
        reg [4:0] state , next_state;
        always @(posedge clk or posedge reset_p)begin
            if(reset_p) state = SCAN0;
            else if(clk_8msec_n)state = next_state;
        end
        always@ *begin
            case(state)
                SCAN0: begin
                    if(row == 0)next_state =SCAN1;
                    else next_state =KEY_PROCESS;
                         end
                SCAN1: begin
                        if(row == 0)next_state =SCAN2;
                        else next_state =KEY_PROCESS;
                        end
               SCAN2: begin
                        if(row == 0)next_state =SCAN3;
                        else next_state =KEY_PROCESS;
                        end
                SCAN3: begin
                        if(row == 0)next_state =SCAN0;
                        else next_state =KEY_PROCESS;
                        end        
                KEY_PROCESS: begin
                        if(row == 0)next_state =SCAN0;
                        else next_state =KEY_PROCESS;
                        end                                             
                default : next_state = SCAN0;        
                        
            endcase
        end
        
        always @(posedge clk or posedge reset_p)begin
            if(reset_p)begin
                key_value =0;
                key_valid =0;
                col = 0;
            end
            else if(clk_8msec_p)begin
               case(state)
                    SCAN0:begin col = 4'b0001; key_valid = 0; end
                    SCAN1:begin col = 4'b0010; key_valid = 0; end
                    SCAN2:begin col = 4'b0100; key_valid = 0; end
                    SCAN3:begin col = 4'b1000; key_valid = 0; end
                    KEY_PROCESS: begin
                        key_valid = 1;
                        case({col,row})
                        8'b0001_0001: key_value = 4'h1;
                        8'b0001_0010: key_value = 4'h5;
                        8'b0001_0100: key_value = 4'h9;
                        8'b0001_1000: key_value = 4'hc;
                        8'b0010_0001: key_value = 4'h2;
                        8'b0010_0010: key_value = 4'h6;
                        8'b0010_0100: key_value = 4'h0;
                        8'b0010_1000: key_value = 4'hd;
                        8'b0100_0001: key_value = 4'h3;
                        8'b0100_0010: key_value = 4'h7;
                        8'b0100_0100: key_value = 4'ha;
                        8'b0100_1000: key_value = 4'he;
                        8'b1000_0001: key_value = 4'h4;
                        8'b1000_0010: key_value = 4'h8;
                        8'b1000_0100: key_value = 4'hb;
                        8'b1000_1000: key_value = 4'hf;
                       
                        endcase
                    end
                endcase
            end
       end
   endmodule   
   
//======================================================================
module timer_1m(
    input clk, reset_p,
    input out_data,
    output reg [5:0] count,
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

    always @(posedge clk or posedge reset_p) begin
        if(reset_p) count=60; 
        else if(out_data) begin
            if(clk_sec_n)  begin
                if(count == 0) count = 0;
                else count = count -1;
            end
         end
      end
    
    
    //count?? timer?ùò ÎπÑÌä∏?àò?äî Í±¥ÎìúÎ¶¨Ï??ïä?úºÎ©? fnd ?ò§Î•∏Ï™Ω 2Í∞úÎßå Í∞íÏùÑ Î≥¥Ïó¨Ï£ºÍ≥†?ã∂?ùÑ?ïå
    wire [15:0] timer_value;
    bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                      .com(com), .seg_7(seg_7));
endmodule


//================================================================
module I2C_txtLCD_top(
    input clk, reset_p,
    input key_valid,
    input open,
    input error,
    output scl,sda);
    
    parameter IDLE          = 6'b00_0001;
    parameter INIT          = 6'b00_0010;
    parameter SEND_PASSWARD = 6'b00_0100;
    parameter SEND_ENTER    = 6'b00_1000;
    parameter SEND_ERROR    = 6'b01_0000;
    parameter SEC_5_WAIT    = 6'b10_0000;
    
    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p),.clk_div_100_nedge(clk_microsec));
    
    reg[21:0] count_microsec;
    reg count_microsec_enable;
    always@(negedge clk or posedge reset_p)begin
        if(reset_p) count_microsec = 0;
        else if(clk_microsec && count_microsec_enable) count_microsec = count_microsec + 1;
        else if(!count_microsec_enable)count_microsec = 0;
    end
    
    reg[7:0] send_buffer;
    reg rs,send;
    
    wire busy; 
    I2C_lcd_send_byte lcd(.clk(clk),.reset_p(reset_p), .addr(7'h27), .send_buffer(send_buffer),.rs(rs),.send(send),.scl(scl),.sda(sda), .busy(busy));
    
    
    reg[5:0] state, next_state;
    always@(negedge clk or posedge reset_p)begin
        if(reset_p) state = IDLE; 
        else state = next_state;
    end
    
    reg init_flag;
    reg[5:0] data_count;
    reg[8*14-1:0]  init_word;
    reg[8*8-1:0]   open_word;
    reg[8*7-1:0]   error_word;
    reg[8*14-1:0]  error2_word;
    always@(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state = IDLE;
            init_flag = 0;
            data_count = 0;
            count_microsec_enable = 0;
            init_word = "Enter Passward";
            open_word = "Welcome!";
            error_word = "ERROR!!";
            error2_word = "Check Passward";
        end
        else begin
            case(state)
                IDLE:begin
                    if(init_flag)begin
                        if(!busy)begin
                            if(key_valid)begin
                                next_state = SEND_PASSWARD;
                                if(open) next_state = SEND_ENTER;
                                else if(error) next_state = SEND_ERROR;
                            end
                            
                        end
                    end
                    else begin
                        if(count_microsec <= 22'd80_000)begin
                            count_microsec_enable = 1;
                        end
                        else begin
                            next_state = INIT;    
                            count_microsec_enable = 0;
                        end
                    end
                end
                INIT:begin
                     if(busy)begin
                        send = 0;
                        if(data_count > 21)begin
                            next_state = IDLE;
                            init_flag = 1;        
                            data_count = 0;    
                            rs = 0;     
                        end
                    end
                    else if(!send) begin //s
                        case(data_count)
                            0: send_buffer = 8'h33;
                            1: send_buffer = 8'h32;
                            2: send_buffer = 8'h28;
                            3: send_buffer = 8'h0F;
                            4: send_buffer = 8'h01;
                            5: send_buffer = 8'h06;
                            6: send_buffer =  init_word[111:104];
                            7: send_buffer =  init_word[103:96]; 
                            8: send_buffer =  init_word[95:88];  
                            9: send_buffer =  init_word[87:80];  
                            10: send_buffer = init_word[79:72];  
                            11: send_buffer = init_word[71:64];  
                            12: send_buffer = init_word[63:56];  
                            13: send_buffer = init_word[55:48];  
                            14: send_buffer = init_word[47:40];  
                            15: send_buffer = init_word[39:32];  
                            16: send_buffer = init_word[31:24];  
                            17: send_buffer = init_word[23:16];  
                            18: send_buffer = init_word[15:8];   
                            19: send_buffer = init_word[7:0];                             
                            20: send_buffer = 8'h06;                             
                            21: send_buffer = 8'hC0;                             
                        endcase
                        if(data_count <= 5) rs=0;
                        else if(data_count > 5 && data_count < 20)rs = 1;
                        else if(data_count > 19)rs = 0;
                        send = 1;
                        data_count = data_count + 1;
                    end
                end
                    SEND_PASSWARD:begin
                    if(busy)begin
                        next_state = IDLE;
                        send = 0;
                        if(data_count >= 9) data_count = 0;
                        else data_count = data_count + 1;
                    end
                    else begin
                        send_buffer = 8'h2A;
                        rs = 1;
                        send = 1;
                    end
                end
                
                SEND_ENTER:begin
                    if(!busy)begin
                        send = 0; 
                        if(data_count > 8)begin
                            next_state = SEC_5_WAIT;      
                            data_count = 0;        
                        end
                    end
                    else if(!send)begin
                        case(data_count)
                            0: send_buffer = 8'h01;
                            1: send_buffer = open_word[63:56];
                            2: send_buffer = open_word[55:48];
                            3: send_buffer = open_word[47:40];
                            4: send_buffer = open_word[39:32];
                            5: send_buffer = open_word[31:24];
                            6: send_buffer = open_word[23:16];
                            7: send_buffer = open_word[15:8];
                            8: send_buffer = open_word[7:0];
                        endcase
                        if(data_count == 0) rs = 0;
                        else if(data_count > 0)rs = 1;
                        send = 1;
                        data_count = data_count + 1;
                    end 
                end
                
                SEND_ERROR:begin
                     if(data_count > 22)begin
                        send = 0;
                        next_state = SEC_5_WAIT;        
                        data_count = 0;       
                    end
                    else if(!send)begin
                        case(data_count)
                            0: send_buffer = 8'h01;
                            1: send_buffer = error_word[55:48];
                            2: send_buffer = error_word[47:40];
                            3: send_buffer = error_word[39:32];
                            4: send_buffer = error_word[31:24];
                            5: send_buffer = error_word[23:16];
                            6: send_buffer = error_word[15:8];
                            7: send_buffer = error_word[7:0];
                            8: send_buffer = 8'hc0;
                            9: send_buffer =  init_word[111:104];
                            10: send_buffer =  init_word[103:96]; 
                            11: send_buffer = init_word[95:88];  
                            12: send_buffer = init_word[87:80];  
                            13: send_buffer = init_word[79:72];  
                            14: send_buffer = init_word[71:64];  
                            15: send_buffer = init_word[63:56];  
                            16: send_buffer = init_word[55:48];  
                            17: send_buffer = init_word[47:40];  
                            18: send_buffer = init_word[39:32];  
                            19: send_buffer = init_word[31:24];  
                            20: send_buffer = init_word[23:16];  
                            21: send_buffer = init_word[15:8];   
                            22: send_buffer = init_word[7:0];       
                        endcase
                        if(data_count == 0) rs = 0;
                        else if(data_count > 0)begin
                            if(data_count == 8)rs = 0;
                            else rs = 1;
                        end    
                        send = 1;
                        data_count = data_count + 1;
                    end 
                end
                SEC_5_WAIT:begin
                    if(count_microsec <= 23'd5_000_000) count_microsec_enable = 1;
                    else begin 
                        count_microsec_enable = 0;
                        next_state = IDLE;
                        init_flag = 0;    
                        data_count = 0;    
                    end
                end
            endcase
        end
    
    end
   
endmodule
