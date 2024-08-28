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
module fnd_cntr(     //컨트롤러  //fnd 0일때 켜진다.  
        input clk, reset_p,
        input [15:0] value,
        output [3:0] com,       //공통단자  //LED의 전원 담당
        output [7:0] seg_7);
        
        ring_counter_fnd rc(clk, reset_p, com);
         // 모듈명         인스턴스명
        reg [3:0] hex_value;     //decoder_7seg에 들어갈 reg
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
//버튼 컨트롤러
//=============================================================
module button_cntr(
        input clk, reset_p,
        input btn,
        output btn_pedge, btn_nedge);
        
         reg[20:0] clk_div = 0; 
         always @(posedge clk)clk_div = clk_div +1;
       
       
         //채터링 과정 스타트
         wire clk_div_nedge;  
    edge_detector_n ed(.clk(clk), .reset_p(reset_p),
                                               .cp(clk_div[16]), .n_edge(clk_div_nedge)); 
                      
                      
         //채터링 잡는 과정                         
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
// T플립플롭 (상승엣지)
/////////////////////////////////////////////////////////////////////////////////////////////////////////
module T_flip_flop_positive(
    input clk, reset_p,
    input t,
    output reg q
);
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) q = 0;
        else begin
            if (t) q = ~q; // t가 1이면 q는 반전
            else q = q; // 생략 가능
        end 
    end
     
endmodule
//=======================================================================
module clock_div_100(
       input clk, reset_p,
       output clk_div_100,     
       output clk_div_100_nedge);
       
       reg  [6:0] cnt_sysclk;       //sysclk :시스템 클락   10-6
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_sysclk =0;
          
          else  begin
              if(cnt_sysclk >= 99) cnt_sysclk = 0; 
             else cnt_sysclk = cnt_sysclk +1;
            
            end  
       end   
                assign clk_div_100 = (cnt_sysclk < 50) ? 0 : 1;     
                
    edge_detector_n ed1(        //다운 엣지 디텍터에 100분주 클럭+다운엣지    
                                                         //전에 코드에서 불러오기
        .clk(clk),
        .reset_p(reset_p),    //(엣지는 ck끝나고 1클럭뒤에 나온다)
        .cp(clk_div_100),  
        .n_edge(clk_div_100_nedge));
      
    
endmodule

//=======================================================================

module clock_div_60(
       input clk, reset_p,
       input clk_source,            //1us clock
       output clk_div_60,     //100분주(기)
       output clk_div_60_nedge);
       
       reg  [9:0] cnt_clksource;       //sysclk :시스템 클락       10-9
     //부족하면 늘려야하지만 넘치면 줄이는건 선택이다.
     // 고칠꺼면 5로 바꾸면 된다.
     // 주로 integer cnt_clksource로 쓰기도한다 (32비트)
     // 32비트를 넘어가면 저렇게 쓰기도 설정해줘야한다.
     
     
       wire clk_source_nedge;         //에지 디텍터 하강엣지 불러오기
               edge_detector_n ed1(.clk(clk), 
                                                        .reset_p(reset_p),    
                                                        .cp(clk_source),  
                                                        .n_edge(clk_source_nedge));
       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_clksource =0;       //초기값 세팅
          
          else  if(clk_source_nedge) begin        //clk_source가 1들어올때마다
                   if(cnt_clksource >= 59) cnt_clksource = 0; 
               else cnt_clksource = cnt_clksource +1;         //1us에 1씩증가해서 1ms로
            
            end  
       end   
                assign clk_div_60 = (cnt_clksource < 30) ? 0 : 1;     
                
    edge_detector_n ed2(              //다운 엣지 디텍터에 100분주 클럭+다운엣지
        .clk(clk), .reset_p(reset_p),    //(엣지는 ck끝나고 1클럭뒤에 나온다)
        .cp(clk_div_60),  .n_edge(clk_div_60_nedge));
    
endmodule

//==============================================================================

module clock_div_1000(
       input clk, reset_p,
       input clk_source,            //1us clock
       output clk_div_1000,     //100분주(기)
       output clk_div_1000_nedge);
       
       reg  [9:0] cnt_clksource;       //sysclk :시스템 클락       10-9
     
       wire clk_source_nedge;         //에지 디텍터 하강엣지 불러오기
               edge_detector_n ed1(.clk(clk), 
                                                        .reset_p(reset_p),    
                                                        .cp(clk_source),  
                                                        .n_edge(clk_source_nedge));
       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_clksource =0;       //초기값 세팅
          
          else  if(clk_source_nedge) begin        //clk_source_nedge가 1들어올때마다
                   if(cnt_clksource >= 999) cnt_clksource = 0; 
               else cnt_clksource = cnt_clksource +1;         //1us에 1씩증가해서 1ms로
            
            end  
       end   
                assign clk_div_1000 = (cnt_clksource < 500) ? 0 : 1;     
                
    edge_detector_n ed2(              //다운 엣지 디텍터에 100분주 클럭+다운엣지
        .clk(clk), .reset_p(reset_p),    //(엣지는 ck끝나고 1클럭뒤에 나온다)
        .cp(clk_div_1000),  .n_edge(clk_div_1000_nedge));
    
endmodule
//==============================================================
//로드가 가능한 60진 카운터 (156~160)
//==============================================================
module loadable_counter_bcd_60(
      input clk,reset_p,
      input clk_time,
      input load_enable,
      input [3:0] load_bcd1, load_bcd10,
      output reg [3:0] bcd1, bcd10); //1의 자리 , 10의 자리
      
      wire clk_time_nedge;
          edge_detector_n ed_clk(.clk(clk), .reset_p(reset_p),   
                                     .cp(clk_time),  .n_edge(clk_time_nedge));
      
      always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin       //리셋 값 
            bcd1 = 0;
            bcd10= 0;
        end
        else begin
                            //변경점
                            //bcd1과 10의 로드된값을 다시 넣는다.
                    if(load_enable)begin
                        bcd1 = load_bcd1;
                        bcd10 = load_bcd10;
                    end   
                        else if(clk_time_nedge) begin
                            if(bcd1 >= 9) begin
                                bcd1 =0;
                          if(bcd10 >= 5) bcd10 = 0;       //60분주니까 60이면 0으로 가자
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
    input rd_wr,        //주소 읽거나 쓰거나
    input [7:0] data,
    input comm_go,  //이게 0->1들어오면 통신시작
    output reg  scl, sda,
    output reg [15:0] led_debug);
     

    
    
    parameter IDLE =  7'b000_0001;
    parameter  COMM_START=  7'b000_0010;
    parameter  SEND_ADDR=  7'b000_0100;
    parameter  RD_ACK=  7'b000_1000;
    parameter  SEND_DATA=  7'b001_0000;
    parameter  SCL_STOP=  7'b010_0000;      //클락 스탑 (속도 100KHZ정도) 10us
    parameter  COMM_STOP=  7'b100_0000;


   //CLOCK US       
   wire clk_usec;
    //us초
   clock_div_100 usec_clk( .clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));     

   //EDGE_DETECTOR  
   wire scl_pedge, scl_nedge, comm_go_pedge;
 
   edge_detector_n SCL_P_N(.clk(clk), .reset_p(reset_p), .cp(scl),  
                                                   .p_edge(scl_pedge), .n_edge(scl_nedge));
  
   edge_detector_n COMM_GO_P(.clk(clk), .reset_p(reset_p), .cp(comm_go),  
                                                         .p_edge(comm_go_pedge));


    //블로그 ADDRESS PACKET  FORMAT 참조
    //10us사용 하는데 en신호를 추가한 개념이다.(SCL제어)
    reg [2:0] count_usec5;
    reg scl_e;  //clock_enable 
    always @(posedge clk or posedge reset_p) begin
        if(reset_p)  begin
            count_usec5 = 0; 
            scl =1;         //scl은 처음에 1
        end
        
        else if(scl_e)begin         //scl_e는 scl 이 1일떄만 동작 
            if(clk_usec) begin
                if(count_usec5 >= 4)begin
                    count_usec5 =0;
                    scl = ~scl;
                end
                else  count_usec5 = count_usec5 +1; 
            end
        end   
        
            else if(!scl_e) begin    //scl_e 이 0이라면
                scl =1;                 //scl은 끝날떄 1로 끝난다.
                count_usec5 = 0;
            end   
    end                                               
   
   
   //state, 주소 선언
    reg [6:0] state ,next_state;     
    wire [7:0] addr_rw;
    assign addr_rw = {addr, rd_wr}; //주소, Read/Write 합쳐서 8비트 
    
    reg [2:0] cnt_bit;
    reg stop_flag;  //데이터 보내고 나면 1로(RD_ACK에서 사용)
    
    //MAIN CODE(프로토콜 구현)
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state = IDLE;
        else  state = next_state;
    end
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state =IDLE;
            scl_e =0;   //clk 은 0으로
            sda =1;
            cnt_bit =7; //최상위 비트 보낼꺼니까 7로 초기화
            stop_flag =0; 
            led_debug = 0;  
        end
        else begin
            case(state)
                IDLE : begin
                led_debug[0] = 1;
                    scl_e =0;
                    sda =1;
                    if(comm_go_pedge) next_state = COMM_START;   //comm_go가 1들어오면 넘어감
                end
                
                COMM_START : begin  //SCL은 1인상태에서 SDA가 0으로 떨어짐, 통신시작>>clock움직임
                    led_debug[1] = 1;
                    sda =0;
                    scl_e =1;   //clock 움직임 >>5us후에 0이 된다.
                    next_state = SEND_ADDR;
                end 
                
                SEND_ADDR : begin
                led_debug[2] = 1;
                    if(scl_nedge) sda =addr_rw[cnt_bit]; //처음에 7번비트 준다.
                    else if(scl_pedge) begin
                        if(cnt_bit ==0) begin 
                            cnt_bit =7;
                            next_state =RD_ACK;  //0이 되면 next_state로
                        end
                        else cnt_bit = cnt_bit -1; //p_edge에선 아무것도 안하니까 그다음비트에서 뺄꺼 미리 준비
                    end                                          //즉 7부터 0까지  쭉 빼는 반복문 설정한거임
                end 
                
                RD_ACK : begin  //RD_ACK다음은 SEND_DATA로 갈지 SCL_STOP으로 갈지 정해줘야함(2가지 경우)
                    led_debug[3] = 1;
                    if(scl_nedge) sda ='bz; 
                    else if(scl_pedge) begin        //pedge들어오면 1클락 끝난거임, clk은 보내야하니까 씀
                        if(stop_flag) begin //초기화 값은 0
                            stop_flag =0;   //SCL 끝과정
                            next_state =SCL_STOP;
                        end
                        else begin
                            stop_flag =1;   // SCL 중간과정 
                            next_state = SEND_DATA;
                        end
                    end
                end 
                
                SEND_DATA : begin
                led_debug[4] = 1;
                if(scl_nedge) sda =data[cnt_bit]; //처음에 7번비트 준다.
                    if(scl_pedge) begin
                        if(cnt_bit == 0) begin 
                            cnt_bit =7;
                            next_state =RD_ACK; //0이 되면 next_state로
                        end
                        else cnt_bit = cnt_bit -1; //p_edge에선 아무것도 안하니까 그다음비트에서 뺄꺼 미리 준비
                    end         
                end 
                
                SCL_STOP : begin
                 led_debug[5] = 1;
                    if(scl_nedge) sda =0;
                    else if(scl_pedge) next_state = COMM_STOP;
                end 
                
                COMM_STOP : begin   //high 상태에서 조금있따가 준다.
                    led_debug[6] = 1;
                    if(count_usec5 >= 3)begin//3us만 기다리다가
                        scl_e =0;   //  클락이 high로 유지되며 
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
    input rs, send,     //send가 1일떄 comm_go에 1떄려서 상위 4비트 보내고/ enable 하위 4비트보내고 /enable
    input [7:0] send_buffer,    //data임
    output scl, sda,
    output reg busy  //busy가 0일때만 send를 보낼수있게 (시간이 좀 걸리니까)
);


    //PARAMETER
                    //NIBBLE= 4비트
    reg [5:0] state, next_state;
    parameter IDLE                                        = 6'b00_0001; 
    parameter SEND_HIGH_NIBBLE_DISABLE = 6'b00_0010;//상위 4비트를 보낼건데 disable로 한번보내고
    parameter SEND_HIGH_NIBBLE_ENABLE  = 6'b00_0100; //상위 4비트를 보낼건데 enable로 한번보내고
    parameter SEND_LOW_NIBBLE_DISABLE  = 6'b00_1000;//하위 4비트를 보낼건데 disable로 한번보내고
    parameter SEND_LOW_NIBBLE_ENABLE   = 6'b01_0000;//하위 4비트를 보낼건데 enable로 한번보내고
    parameter SEND_DISABLE                        = 6'b10_0000;//disable신호를 보내서 값을 못읽게 한다.
                          // LCD기준 1바이트(8비트) 받는거임 
    
    reg [7:0] data;
    reg comm_go; //이게 0->1들어오면 통신시작
    I2C_controller(.clk(clk) , .reset_p(reset_p) ,
                           .addr(addr) , .rd_wr(0) , // .rd_wr(0) : 쓰기만
                           .data(data), .comm_go(comm_go), 
                           .scl(scl), .sda(sda), .led_debug(led_debug));
    
   //CLOCK US(us초)
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
                    if(count_usec <= 22'd200)  begin            //200us //start, I2C과정(18)  , stop 해서 여유롭게 200us
                        data= {send_buffer[7:4], 3'b100, rs};   // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4까지)
                        comm_go =1;                                     //BT만 1인거임
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_HIGH_NIBBLE_ENABLE;   
                    end
                 end 

                 SEND_HIGH_NIBBLE_ENABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C과정(18)  , stop 해서 여유롭게 200us
                        data= {send_buffer[7:4], 3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4까지)
                        comm_go =1;                  //BT만 1인거임
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_LOW_NIBBLE_DISABLE;   
                    end
                  end 
                 
                 SEND_LOW_NIBBLE_DISABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C과정(18)  , stop 해서 여유롭게 200us
                        data= {send_buffer[3:0], 3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4까지)
                        comm_go =1;                  //BT만 1인거임
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_LOW_NIBBLE_ENABLE;   
                    end
                 end                      
                 
                 SEND_LOW_NIBBLE_ENABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C과정(18)  , stop 해서 여유롭게 200us
                        data= {send_buffer[3:0], 3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4까지)
                        comm_go =1;                  //BT만 1인거임
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = SEND_DISABLE;   
                        end
                 end
                 
                 SEND_DISABLE : begin
                    if(count_usec <= 22'd200)  begin  //200us //start, I2C과정(18)  , stop 해서 여유롭게 200us
                        data= {send_buffer[3:0], 3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS (13~ 4까지)
                        comm_go =1;                  //BT만 1인거임
                        count_usec_e =1;
                    end
                    else begin
                        count_usec_e =0;
                        comm_go =0;
                        next_state = IDLE;   
                        busy =0;    //IDLE로 갈떄 busy가 1 , 나머지에선 0
                     end               //다 보내고 나면 busy가 0이라고 생각 
                 end
            endcase
          end
      end
endmodule

//======================================================
module servo_motor(
    input clk,reset_p,
    input open,
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

        else if(open) begin
            duty =35;
        end

        else if(close) begin
            duty =12;
        end
      end

    pwm_Nstep_freq #(.duty_step(400), .pwm_freq(50))
    pwm_b(.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(servo_motor_pwm));

    wire [15:0] duty_bcd;
 

endmodule


//==========================================================================
module pwm_Nstep_freq     //공식화
#( parameter sys_clk_freq = 100_000_000,    
    parameter pwm_freq = 10_000,                //led는 10000hz에서 동작
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
                                                     //같다 까지 끼면 duty가 0일때도 1이 출력된다,  
endmodule


//===========================================================
//초음파센서 slack 잡기
//시스템설계할땐 나누기 하면 꼬임
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

        //ms단위
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
      
         reg[21:0] count_usec;  //3초 카운트하기 위해 22비트
         reg count_usec_en;   //enable
         
         reg[21:0] echo_time;   //나눗셈 처리를 위해 시간을 저장할 변수
         
          always @(posedge clk or posedge reset_p)begin
             if(reset_p) count_usec =0;
               else if(clk_usec && count_usec_en) count_usec = count_usec +1;
               else if(!count_usec_en) count_usec = 0;               
          end       
       
       //S_IDLE 
       //n에서 state바꾸고
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
                if(count_usec < 22'd1_000_000)  begin   //1초
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
                    distance = cm;      //distance 값을 value값에 넣었으니까 cm
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
     //나눗셈 제어(먹스로 구성) 
     always @(posedge clk or posedge reset_p)begin
        if(reset_p)distance =0;
        else begin  //58의 배수
      
        end
     end
endmodule

//===============================================================
//58분주 엣지트리거를 사용하지 않고 쓰느방법
//===============================================================
module clock_div_58(
       input clk, reset_p,
       input clk_usec, count_en,      
       output reg[11:0] cm);
       
       reg  [5:0] count;       
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) begin       //초기값 세팅
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
    
    
    //count와 timer의 비트수는 건드리지않으며 fnd 오른쪽 2개만 값을 보여주고싶을때
    wire [15:0] timer_value;
    bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                      .com(com), .seg_7(seg_7));

endmodule


//================================================================
module I2C_txtLCD_top(
    input clk, reset_p,
    input key_valid,
    input key_value,
    input start_stop,
    output scl,sda);
    
    parameter IDLE = 5'b0_0001;
    parameter INIT = 5'b0_0010;
    parameter SEND_PASSWARD = 5'b0_0100;
    parameter SEND_ENTER = 5'b0_1000;
    parameter SEND_ERROR = 5'b1_0000;
    
    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p),.clk_div_100_nedge(clk_microsec));
    
    reg[21:0] count_microsec;
    reg count_microsec_enable;
    always@(negedge clk or posedge reset_p)begin
        if(reset_p) count_microsec = 0;
        else if(clk_microsec && count_microsec_enable) count_microsec = count_microsec + 1;
        else if(!count_microsec_enable)count_microsec = 0;
    end
    
//    wire[3:0]btn_pedge;
//    button_cntr btn0(.clk(clk), .reset_p(reset_p),.btn(btn[0]), .btn_posedge(btn_pedge[0]));
//    button_cntr btn1(.clk(clk), .reset_p(reset_p),.btn(btn[1]), .btn_posedge(btn_pedge[1]));
//    button_cntr btn2(.clk(clk), .reset_p(reset_p),.btn(btn[2]), .btn_posedge(btn_pedge[2]));
//    button_cntr btn3(.clk(clk), .reset_p(reset_p),.btn(btn[3]), .btn_posedge(btn_pedge[3]));
    
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
    reg[3:0] cnt_string;
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
            cnt_string = 14;
        end
        else begin
            case(state)
                IDLE:begin
                    if(init_flag)begin
                        if(!busy)begin
                            if(key_valid)begin
                                next_state = SEND_PASSWARD;
                                //if(start_stop == 1) next_state = SEND_ENTER;
                                //else if(start_stop == 0) next_state = SEND_ENTER;
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
                    if(busy)begin
                        send = 0;
                        if(data_count > 8)begin
                            if(count_microsec <=23'd5_000_000)
                                count_microsec_enable = 1;
                            else begin
                                next_state = IDLE;
                                init_flag = 0;
                                data_count = 0;
                            end
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
                     if(busy)begin
                        send = 0;
                        if(data_count > 23)begin
                            if(count_microsec <=23'd5_000_000)
                                count_microsec_enable = 1;
                            else begin
                                next_state = IDLE;
                                init_flag = 0;
                                data_count = 0;
                            end
                        end
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
            endcase
        end
    
    end
    
    
    
endmodule