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
    output reg busy,  //busy가 0일때만 send를 보낼수있게 (시간이 좀 걸리니까)
    output [15:0] led_debug);


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
    reg [7:0] send_buffer;      //input이니까 reg 로 선언해서 받음(데이터받는 레지스터)
    reg rs,send;
    wire busy;
    I2C_lcd_send_byte(.clk(clk) , .reset_p(reset_p) , .addr(7'h27) ,.rs(rs), .send(send), .send_buffer(send_buffer), 
                                  .scl(scl), .sda(sda), .busy(busy),.led_debug(led_debug));
    
    
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) state = IDLE;
        else state = next_state;
    end
    
    
    reg INIT_flag;  //했는지 안했는지 확인용
    reg [2:0]cnt_data;
    reg [5*8-1 :0] hello; //5글자(8비트)짜리 변수 (-1은  0부터 시작하니까 뺌)([] 안은 다 상수다)
    reg [3:0] cnt_string; //글자수를 세는 변수(여기선 hello 5개를 셈)
    reg [2:0]command2_cnt_data;
    reg [1*8-1 :0]h;
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = IDLE;    
            INIT_flag =0;
            count_usec_e =0;
            cnt_data =0;
            hello = "HELLO";    //글자는 이런식으로 초기화( ""안에 hello의 아스키 코드 값이 들어간다)
            h = "H";
            cnt_string =5;
            command2_cnt_data =0;
        end
        else begin
            case(state)
                IDLE : begin            
                    if(INIT_flag) begin
                        if(btn_pedge[0]) next_state = SEND_DATA; //(INIT_flag)가 1이면 버튼0번 눌러서SEND_BYTE
                        if(btn_pedge[1]) next_state = SEND_COMMAND;
                        if(btn_pedge[2]) next_state = SEND_STRING;
                        if(btn_pedge[3]) next_state = SEND_COMMAND2;
                    end
                    else begin
                        if(count_usec <= 22'd80_000) begin      //80ms
                            count_usec_e =1;
                        end
                        else begin
                             next_state = INIT;     // INIT_flag가 0이면 INIT으로 가기
                                                             // 처음 한번만 INIT으로 가고 다시 갈일 없다.
                             count_usec_e =0;
                        end
                    end
                end
                
                INIT : begin    //처음 한번만 하면 상관없다.(INIT_flag 로 체크)
                                        //처음엔 busy가 0이니까 else문에서 6번에 거쳐서 cnt
                    if(busy) begin  // busy는 send_pedge가 들어올떄 1이된다.
                          send =0;
                          if(cnt_data >= 6) begin
                              next_state = IDLE;
                              INIT_flag = 1;  // 초기화 
                              cnt_data =0;
                              rs =0;
                          end
                      end
                      else if(!send)begin 
                      //send가 0이 아닌경우엔 1부터 들어올수도있음
                      //busy는 send의 상승엣지 에서 발생하고 clk도 n_edge에 의해 동작하기 때문에 
                      // 클럭 차이가 생길수 있다. 그렇게 때문에 send가 1부터 시작하는 경우가 생길수 있고
                      //그걸 막기 위해서 !send라는 조건을 붙혀서 send가 1로 왔을때를 방지한다. 
                                 //즉 send =0, busy =0 일떄
                          case(cnt_data)  //초기화 코드 6개 보냄
                              0 : send_buffer = 8'h33;  //rs =0;send를 endcase에서 1을 주면 sned_buffer
                              1 : send_buffer = 8'h32;  //그 다음 상태로 넘어가는게 send_pedge를 보고 넘어가기때문에 1클럭 뒤에 넘어감
                                                                    // 그러므로 send = 0 을 주면 일단 busy가 1이 아니므로 아래부터 날라가게 된다.
                              2 : send_buffer = 8'h28;  //NF N: 1 F: 0  0011
                              3 : send_buffer = 8'h0f;    //display on
                              4 : send_buffer = 8'h01; // display clear 
                              5 : send_buffer = 8'h06;
                         endcase
                        rs =0;  //초기화 코드를 입력하는것이기에 rs =0을 해준다.
                        send =1;    //보내기
                        cnt_data = cnt_data +1;
                    end                 
                end
                
                
                SEND_DATA : begin   //idle에서 btn0누르면 바이트 보낸다.
                    if(busy) begin //busy상태에서 넘어가므로 busy를 체크하는 코드를 생성한다.
                        next_state = IDLE;  // send가 1되어서 busy가 1되면 이쪽으로 넘어온다.
                        send = 0;
                        if(cnt_data >= 9) cnt_data =0;
                        else cnt_data =cnt_data +1;
                    end
                    else begin
                        send_buffer = "A" + cnt_data;  //대문자 A의 아스키 값으로 간다.
                            //A를 0으로 바꾸면 숫자들이 출력
                        rs =1; //데이터 보낼것이므로 rs =1;
                        send =1;    //busy가 1될거임 
                    end
                end               
                    
                    
     //SEND_DATA에서 RS를 0주고(데이터 저장작업) SEND_BUFFER에 shift에 작업 코드 생성
                    SEND_COMMAND : begin   //idle에서 btn0누르면 바이트 보낸다.
                    if(busy) begin //busy상태에서 넘어가므로 busy를 체크하는 코드를 생성한다.
                        next_state = IDLE;  // send가 1되어서 busy가 1되면 이쪽으로 넘어온다.
                        send = 0;
                    end
                    else begin
                        send_buffer =  8'h18;  //좌 시프트 
                          //cursor on display shift에서 (1(DB5)/ 1000(DB4321))
                        rs =0;  //RS가 0이면 쓰거나 데이터를 저장하는 작업을 수행하고 보내는것이 아님 
                        send =1;    
                    end
                end           
                
                
                
                    // 마이너스(-)로 할거면 IDLE 초기값을 cnt_string=5로 정하고 기준값을 바꿔준다.
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
                                              // [8*cnt_string-1: 8*(cnt_string-1);이건데 변수를 못써서 상수를 써야함  
                                              // 즉 위에서 만든 cnt_string의 변수값으로 상수 설정
                              4 : send_buffer = hello[31: 24];  
                              3 : send_buffer = hello[23: 16];  
                              2 : send_buffer = hello[15: 8];   
                              1 : send_buffer = hello[7: 0]; 
                         endcase
                             rs =1;     //여긴 LCD에 보낼꺼니가 rs=1한다.
                             send =1;    //보내기
                             cnt_string = cnt_string - 1;
                      end                 
                     end             

                //SEND_COMMAND를 case문으로 변경후 다음상태를 SEND_DATA로 보내는 과정
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
    
        reg [31:0] clk_div;     //속도조절\
    always @(posedge clk or posedge reset_p) begin
        if(reset_p)clk_div = 20;
        else clk_div = clk_div +1;
    end

    wire clk_div_26_nedge;    //p edge   //누르면 1 한번더 누르면 0 >> T플립플롭 사용                                          
    edge_detector_n ed(.clk(clk), .reset_p(reset_p),
                                                      .cp(clk_div[23]), .n_edge(clk_div_26_nedge));    
                                                      
    reg [6:0] duty;       // duty 레지스터의 크기를 8비트로 설정
    reg up_down;        // 방향 제어를 위한 플래그
    reg duty_min, duty_max;
 
 wire btn_0, btn_1, btn_2;
      button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_0));
      button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_1));
      button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_2));
 
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duty = 12 ;       // 초기화 1ms (5% 듀티 사이클)
            up_down = 1;  // 초기 방향 설정 (0: 증가, 1: 감소)
            duty_min =12;
            duty_max = 50;
        end
        else if (clk_div_26_nedge) begin // 20ms 주기
            if (up_down) begin
                if (duty <50)  // 2ms (10%)에 도달하지 않았다면 증가
                    duty = duty + 1;
                else
                    up_down = 0;  // 2ms에 도달하면 방향을 감소로 변경
            end
            else begin
                if (duty > 12)  // 1ms (5%)에 도달하지 않았다면 감소
                    duty = duty - 1;
                else
                    up_down = 1;  // 1ms에 도달하면 방향을 증가로 변경
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

module keypad_cntr_FSM(
    input clk,reset_p,
    input [3:0] row,        //줄
    output reg [3:0] col,      //열
    output reg [3:0] key_value,
    output reg key_valid);     
    
    parameter SCAN0               = 5'b00001;
    parameter SCAN1               = 5'b00010;
    parameter SCAN2               = 5'b00100;
    parameter SCAN3               = 5'b01000;
    parameter KEY_PROCESS = 5'b10000;
    

        reg[19:0] clk_div;
    always @(posedge clk)clk_div = clk_div +1;      //순서회로
  
    wire clk_8msec_n, clk_8msec_p;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(clk_div[19]), 
                                                  .p_edge(clk_8msec_p), .n_edge(clk_8msec_n)); 
      
      reg[4:0] state, next_state;

        always@(posedge clk or posedge reset_p)begin      
            if(reset_p)state =SCAN0;
            else if(clk_8msec_p)state =next_state;
        end

        always@(*)begin     //조합회로    FSM
            case(state) 
                SCAN0 : begin
                        if(row ==0) next_state =  SCAN1;    //키입력이 있으면 키 프로세스
                        else next_state = KEY_PROCESS;          //next_state같은 = 앞은 레지스터(플립플롭)으로 사용된다.
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
                    case(state)     //각각의 상태 구현
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
                                    8'b0001_0001 : key_value = 4'h0;        //값을 바꾸고 바로 읽음
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
    
    
    //count와 timer의 비트수는 건드리지않으며 fnd 오른쪽 2개만 값을 보여주고싶을때
    wire [15:0] timer_value;
    bin_to_dec timer(.bin({6'b0, count[5:0]}), .bcd(timer_value)); 
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(timer_value),
                                      .com(com), .seg_7(seg_7));

endmodule
