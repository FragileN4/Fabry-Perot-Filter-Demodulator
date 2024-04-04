`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/15 15:13:51
// Design Name: 
// Module Name: buf_start_ctrl
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//////////////////////////////////
//  FP��׼�߲����궨ģ�� //////////
//////////////////////////////////////////////////////////////////////////////////


module FP_peakfind2 #(parameter FIT_NUM = 16'd21)(
    input               sys_clk,
    input               sys_rst_n,
    input               AD_start,
    input               data_wr_end,
    input               polyfit_en_dout,
    input[31:0]         result,
    input               pi_flag,
    input[15:0]         pi_addr,
    input[15:0]        	pi_data,
    //ȫ���׻ᳬ����Դ�ķ�Χ�����Ѱ��Сֵ���ܻ�Ѱ����Դ��Χ��ĵ������Ʒ���׵���С�㣬��Ҫȷ��Ѱ��Сֵ�ĵ�ַ��Χ
    output reg[15:0]    min_str_addr,           
    output reg          min_str_en,  
    output reg[15:0]    min_end_addr,
    output reg[7:0]     warning,    
    //ÿ������������,��ϵ��ݶ�Ϊ21
    output reg          peak_fit_en,
    output reg[15:0]    fit_str_addr,       //��Ͽ�ʼ��ַ
    input[31:0]          peak_i,  
    input                peak_i_en,                        
    
    output [31:0]       wave_result,
    output              cal_en_dout
);
localparam           AMP_STA        =  16'd2450;  //Ѱ�ҵ�-1����,1527.24��λ���õ��������ֵ��2600�� ����:2820
localparam           PEAK_END_CNT   =  16'd48;    //Ѱ��49���弴�ɡ�48��     ����:45      

localparam           S_idle     =  3'b000;
localparam           S_sta      =  3'b111;
localparam           S_down     =  3'b001;
localparam           S_up       =  3'b011;
localparam           S_buf_end  =  3'b010;
localparam           S_peakfind =  3'b110;
localparam           S_end      =  3'b100;

integer             i;

reg[2:0]	         state,next_state;

reg [47:0]           form_reg;
wire[15:0]           data_z_2    = form_reg[47:32];
wire[15:0]           data_z_1    = form_reg[31:16];
wire[15:0]           data_z_0    = form_reg[15:0];
wire                 peak_flag   = pi_flag && data_z_2 <= data_z_1 && data_z_1 > data_z_0;
wire                 valley_flag = pi_flag && data_z_2 >= data_z_1 && data_z_1 < data_z_0;

reg[15:0]            peak_cnt;
reg[31:0]            peak[PEAK_END_CNT:0];

reg[31:0]            sample_peak;
reg[7:0]             left_cnt,right_cnt;
wire[31:0]           left_wave,right_wave;
reg[3:0]             cal_state;
wire                 cal_flag;

assign               cal_flag=cal_state[3];

always @(posedge sys_clk or negedge sys_rst_n) begin    
    if(sys_rst_n==1'b0)begin
        peak_cnt    <= 16'd0;           
        for(i=0;i <= PEAK_END_CNT;i=i+1)begin
            peak[i] <= 32'd0;
        end
    end
    else if(state == S_idle)begin
        peak_cnt    <= 16'd0;           
        for(i=0;i <= PEAK_END_CNT;i=i+1)begin
            peak[i] <= 32'd0;
        end
    end    
    else if(peak_i_en)begin
        peak_cnt       <= peak_cnt + 1'b1;           
        peak[peak_cnt] <= peak_i[15:0] * 10000 + peak_i[31:16]; 
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin    
    if(sys_rst_n==1'b0)
        state <= S_idle; 
    else 
        state <= next_state;
end

always @(posedge sys_clk or negedge sys_rst_n) begin    
    if(sys_rst_n==1'b0)
        form_reg <= 48'd0; 
    else if(pi_flag)
        form_reg <= {form_reg[31:0],pi_data}; 
end

always @(*) begin    
    case(state)
        S_idle:
            if(pi_flag == 1'b1 && pi_data < AMP_STA)    
                next_state = S_sta;
	        else
	            next_state = S_idle;      
        S_sta:
            //if(pi_flag == 1'b1 && pi_data > (AMP_STA + (AMP_STA >> 2)))    
            if(pi_flag == 1'b1 && pi_data > 2600) 
                next_state = S_up;
	        else
	            next_state = S_sta;      	              
	    S_down:
            if(AD_start)
                next_state = S_idle;    	    
            else if(data_wr_end)
                next_state = S_buf_end;    	    
	        else if(valley_flag)
                next_state = S_up;                
	        else
	            next_state = S_down;	
	    S_up:
            if(AD_start)
                next_state = S_idle;    	       	    
	        else if(peak_flag && peak_cnt < PEAK_END_CNT)
                next_state = S_down;              
            else if(peak_flag && peak_cnt == PEAK_END_CNT || data_wr_end)
                next_state = S_buf_end;               
	        else
	            next_state = S_up;	
	    S_buf_end:
            if(AD_start)
                next_state = S_idle;  	    
	        else if(polyfit_en_dout == 1'b1)
                next_state = S_peakfind; 
	        else
	            next_state = S_buf_end;		            
        S_peakfind:
        //����:�����λ�ø������ڷ�Χ��(<peak[0]��>peak[48])״̬���Ῠ�ڵ�ǰ״̬
            if(AD_start)
                next_state = S_idle;  	 
            else if((sample_peak >= peak[left_cnt] && sample_peak <= peak[left_cnt+1]) || sample_peak < peak[0] || sample_peak > peak[PEAK_END_CNT])
                next_state = S_end;    
            else
                next_state = S_peakfind;       	                               
        S_end:
            if(AD_start==1'b1)
                next_state = S_idle;
            else
                next_state = S_end;            
	    default:next_state = S_idle;
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin    
    if(sys_rst_n==1'b0)begin
        sample_peak    <= 32'd0; 
        left_cnt       <= 8'd0;
        right_cnt      <= 8'd0;
        min_str_addr   <= 16'hffff;
        min_str_en     <= 1'b0;
        min_end_addr   <= 16'hffff;
        warning        <= 8'h0;
        peak_fit_en    <= 1'b0;    
        fit_str_addr   <= 16'b0;        
        cal_state      <= 4'b0001;
    end
    else case(state)
        S_idle:begin                       
            sample_peak    <= 32'd0; 
            left_cnt       <= 8'd0;
            right_cnt      <= 8'd0;
            min_str_addr   <= 16'hffff;
            min_str_en     <= 1'b0;
            min_end_addr   <= 16'hffff;
            warning        <= 8'h0;
            peak_fit_en    <= 1'b0;    
            fit_str_addr   <= 16'b0;        
            cal_state      <= 4'b0001;
        end
        S_down:begin       
            min_str_en     <= 1'b0;
            peak_fit_en    <= 1'b0;    
            if(data_wr_end)begin
                warning        <=   8'd1;
                min_end_addr   <=   pi_addr;    
            end            
        end
        S_up:begin
            if(data_wr_end)begin
                warning        <=   8'd1;
                min_end_addr   <=   pi_addr;    
            end
            else if(peak_flag && peak_cnt < PEAK_END_CNT)begin
                peak_fit_en    <= 1'b1;
                fit_str_addr   <= pi_addr - 1'b1; 
                if(peak_cnt == 8'd0)begin
                    min_str_addr   <=   pi_addr - 1'b1; 
                    min_str_en     <=   1'b1;
                end                     
            end
            else if(peak_flag && peak_cnt == PEAK_END_CNT)begin
                peak_fit_en    <= 1'b1;
                fit_str_addr   <= pi_addr - 1'b1; 
                min_end_addr   <= pi_addr - 1'b1; 
            end                  
        end
        S_buf_end:begin
            peak_fit_en    <= 1'b0; 
            if(polyfit_en_dout == 1'b1)
                sample_peak    <= result[15:0] * 10000 + result[31:16];           
        end  
        S_peakfind:begin 
            if(sample_peak > peak[left_cnt+1])
                left_cnt <= left_cnt + 1'b1;
            else if(sample_peak >= peak[left_cnt] && sample_peak <= peak[left_cnt+1])
                right_cnt<= left_cnt + 1'b1; 
        end   
        S_end:begin
            cal_state <= (cal_state << 1);
        end
        default:begin
            sample_peak    <= 32'd0; 
            left_cnt       <= 8'd0;
            right_cnt      <= 8'd0;
            cal_state      <= 4'b0001;        
        end
    endcase
end

refer_wave left_wave_inst (
  .clka(sys_clk),    // input wire clka
  .addra(left_cnt),  // input wire [5 : 0] addra
  .douta(left_wave)  // output wire [31 : 0] douta
);

refer_wave right_wave_inst (
  .clka(sys_clk),    // input wire clka
  .addra(right_cnt),  // input wire [5 : 0] addra
  .douta(right_wave)  // output wire [31 : 0] douta
);

wave_cal wave_cal_inst(
    .sys_clk        (sys_clk        ),
    .sys_rst_n      (sys_rst_n      ),
    .cal_flag       (cal_flag       ),
    .left_wave      (left_wave      ),
    .right_wave     (right_wave     ),    
    .left_peak      (peak[left_cnt] ), 
    .right_peak     (peak[right_cnt]), 
    .sample_peak    (sample_peak    ), 
    .wave_result    (wave_result    ),
    .cal_en_dout    (cal_en_dout    )
);
endmodule
