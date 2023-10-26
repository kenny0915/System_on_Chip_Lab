`timescale 1ns / 1ps

`define SS_IDLE 1'b1
`define SS_DONE 1'b0

`define SM_IDLE 1'b1
`define SM_DONE 1'b0

`define AP_IDLE 2'b00
`define AP_INIT 2'b01
`define AP_DONE 2'b10

module fir 
  #(parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
    )
    (
// AXI-Lite
    output wire                     awready,    // addr write ready
    output wire                     wready,     // data write ready
    input  wire                     awvalid,    // addr write valid
    input  wire                     wvalid,     // data write valid
    input  wire [(pADDR_WIDTH-1):0] awaddr,     // addr write data  
    input  wire [(pDATA_WIDTH-1):0] wdata,      // data write data  
    output wire                     arready,    // addr read ready
    input  wire                     rready,     // data read ready
    input  wire                     arvalid,    // addr read valid
    output wire                     rvalid,     // data read valid
    input  wire [(pADDR_WIDTH-1):0] araddr,     // addr read data
    output wire [(pDATA_WIDTH-1):0] rdata,      // data read data 
    
// AXI-Stream(X[t])
    output wire                     ss_tready,  // fir ready
    input  wire                     ss_tvalid,  // .tb valid
    input  wire [(pDATA_WIDTH-1):0] ss_tdata,   // .tb data input
    input  wire                     ss_tlast,   // .tb addr input
// AXI-Stream(Y[t])
    input  wire                     sm_tready,  // .tb ready
    output wire                     sm_tvalid,  // for valid
    output wire [(pDATA_WIDTH-1):0] sm_tdata,   // fit data output
    output wire                     sm_tlast,   // ifr addr output
// tap RAM
    output wire [3:0]               tap_WE,     // tap RAM write enable
    output wire                     tap_EN,     // tap RAM enable
    output wire [(pDATA_WIDTH-1):0] tap_Di,     // write data to tap RAM
    output wire [(pADDR_WIDTH-1):0] tap_A,      // tap RAM address
    input  wire [(pDATA_WIDTH-1):0] tap_Do,     // read data from tap RAM
// data RAM
    output wire [3:0]               data_WE,    // data RAM write enable
    output wire                     data_EN,    // data RAM enable
    output wire [(pDATA_WIDTH-1):0] data_Di,    // write data to data RAM
    output wire [(pADDR_WIDTH-1):0] data_A,     // data RAM address
    input  wire [(pDATA_WIDTH-1):0] data_Do,    // read data from data RAM
    
    input  wire                     axis_clk,
    input  wire                     axis_rst_n
    );
// save data length
    reg [9:0] data_length;
    wire [9:0] data_length_tmp;
     
    assign data_length_tmp = (awaddr == 32'h0000_0010)?wdata:data_length;       // when not write data_length, keep the original value
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            data_length <= 10'd0;                                               // when reset, data_length is 0
        else
            data_length <= data_length_tmp;
    end
// ap control
    reg [2:0]  ap_ctrl;                                                         // [ap_idle:ap_done:ap_start]
    reg [1:0]  ap_state;
    reg [1:0]  ap_next_state;
    reg  [9:0] tlast_cnt;
    
    always @* begin
        case (ap_state)
            `AP_INIT:
            begin
                if (awaddr == 12'd0 && wdata[0] == 1 && tlast_cnt != data_length) begin
                    ap_next_state = `AP_IDLE;
                    ap_ctrl = 3'b001;                                           // start
                end
                else begin
                    ap_next_state = `AP_INIT;
                    ap_ctrl = 3'b100;                                           // ready to start
                end  
            end
            `AP_IDLE:
            begin
                if (sm_tlast) begin                                             // finish last y[t]
                    ap_next_state = `AP_DONE;
                    ap_ctrl = 3'b010;                                           // done
                end
                else begin
                    ap_next_state = `AP_IDLE;
                    ap_ctrl = 3'b000;                                           // processing: low active

                end
            end
            `AP_DONE:
            begin
                if (araddr == 12'd0 && arvalid == 1'b1 && rvalid == 1'b1) begin
                    ap_next_state = `AP_INIT;
                    ap_ctrl = 3'b010;
                end
                else begin
                    ap_next_state = `AP_DONE;
                    ap_ctrl = 3'b010;
                   
                end
            end
            default:
            begin
                if (awaddr == 12'd0 && wdata[0] == 1 && tlast_cnt != data_length) begin
                    ap_next_state = `AP_IDLE;
                    ap_ctrl = 3'b001; 
                end
                else begin
                    ap_next_state = `AP_INIT;
                    ap_ctrl = 3'b100;
                end 
            end
        endcase
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_state <= `AP_INIT;
        else
            ap_state <= ap_next_state;
    end

// counter

    // counter for x[t]: when processing, count from 0 to 10 every clock cycle
    reg  [3:0] cnt_x;
    wire [3:0] cnt_x_tmp;
    
    assign cnt_x_tmp = (cnt_x != 4'd10)? cnt_x+1 : 4'd0;

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2])
            cnt_x <= 4'd10;
        else
            cnt_x <= cnt_x_tmp;
    end
    
    // counter for y[t]: when processing, count: -15 -14 -13 ... 0 1 2 3 ... 10 0 1 2 3 ... 10...
    // we need 3 cycle to do the pipeline and 11 cycle to caculate first y[t], so we start from -15.  
    
    reg  [4:0] cnt_y;
    wire [4:0] cnt_y_tmp;
    
    assign cnt_y_tmp = (cnt_y != 6'd10 && ap_ctrl[2] == 1'b0)? cnt_y + 1'b1 : 5'd0;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2]) 
            cnt_y <= 5'd0 - 5'd15;
        else
            cnt_y <= cnt_y_tmp;
    end
    
    // counter for initialize data RAM to 0
    
    wire [5:0] next_init_addr;
    reg  [5:0] init_addr;
    
    assign next_init_addr = (init_addr == 6'd44)? init_addr : init_addr + 6'd4;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            init_addr <= -6'd4;
        else
            init_addr <= next_init_addr;
    end  
    
    // counter for shift data RAM read pointer: count up every 11 cycles
    
    reg [3:0] shift;
    reg [3:0] shift_tmp;
    
    always @* begin
        if (ap_ctrl[2] == 1'b0) begin
            if (cnt_x == 4'd10)
                if (shift != 4'd10)
                    shift_tmp = shift + 1'b1;
                else
                    shift_tmp =  4'd0;
            else
                shift_tmp = shift;
        end
        else
            shift_tmp = 4'd0;
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            shift <= 4'd0;
        else
            shift <= shift_tmp;
    end
    
    // counter for sm_tlast: count to 600th y[t]
    
    wire [9:0] tlast_cnt_tmp;
    
    assign tlast_cnt_tmp = (sm_tvalid == 1'b1)? tlast_cnt + 1'b1 : tlast_cnt;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) 
            tlast_cnt <= 10'd0;
        else
            tlast_cnt <= tlast_cnt_tmp;
    end

// axi_lite protocol
    assign awready = 1'b1;
    assign arready = ~(wvalid | awvalid); 
    assign wready  = 1'b1;
    
    reg rvalid_tmp;
    
    assign rvalid = rvalid_tmp;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin 
        if (!axis_rst_n)
            rvalid_tmp <= 1'b0;
        else
            rvalid_tmp <= arvalid;                                                   // rvalid trigeer when arvalid is sample
    end
    
    assign rdata = (araddr[7:0] == 8'd0) ? ap_ctrl : tap_Do;
    
// tap RAM
    wire [(pADDR_WIDTH-1):0] tap_araddr;
    
    assign tap_EN = 1;
    assign tap_WE = ((awvalid && wvalid == 1) && (awaddr != 0))? 4'b1111 : 4'b0000;
    assign tap_A  = (awvalid == 1) ? awaddr[5:0] : tap_araddr[5:0];                  // if write valid than use write addr. 
                                                                                     // if not, it means we should read data, than use read address we generated
    assign tap_araddr = (ap_ctrl[2] == 1'b0)? 12'h080 + 4*cnt_x : araddr[5:0];
    assign tap_Di = wdata;
    
// axi_stream:ss
    assign ss_tready = (init_addr != 6'd44 || cnt_x == 4'd0) ? 1'b1 : 1'b0;
    
// axi_stream:ss finite state machine
    reg ss_state;
    reg ss_next_state;
    reg ss_idle;
    
    always @* begin
        case (ss_state)
        `SS_IDLE:
        begin
            if (ss_tlast == 1) begin
                ss_next_state = `SS_DONE;
                ss_idle = 1;
            end
            else begin
                ss_next_state = `SS_IDLE;
                ss_idle = 1;
            end
        end
        `SS_DONE:
        begin
            if (ss_tvalid == 1) begin
                ss_next_state = `SS_IDLE;
                ss_idle = 1;
            end
            else begin
                ss_next_state = `SS_DONE;
                ss_idle = 0;
            end
        end
        default:
        begin
            if (ss_tvalid == 1) begin
                ss_next_state = `SS_IDLE;
                ss_idle = 1;
            end
            else begin
                ss_next_state = `SS_DONE;
                ss_idle = 0;
            end
        end
        endcase
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            ss_state <= `SS_DONE;
        else
            ss_state <= ss_next_state;
    end
// data_RAM

    reg  [(pDATA_WIDTH-1):0] data_ff;
    wire [5:0] data_A_tmp;
    
    assign data_EN = ss_tvalid;
    assign data_WE = (ss_tready == 1 & ss_tvalid == 1)? 4'b1111 : 4'b0000;
    assign data_A  = (ap_ctrl[2] == 1 && init_addr != 6'd44)? init_addr : data_A_tmp;// initialize data RAM before processing                   
    assign data_A_tmp = (cnt_x <= shift)? 4 * (shift - cnt_x) : 4 * (11 + shift - cnt_x);
                                                                                     // read data address pointer 
    assign data_Di = ss_tdata;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin                           // load ss_tdata to FF 
        if (!axis_rst_n) begin
            data_ff <= 32'd0;
        end
        else begin
            data_ff <= ss_tdata;
        end
    end
    
    wire [(pDATA_WIDTH-1):0]x_data;
    
    assign x_data = (cnt_x == 4'd0)? data_ff: data_Do;

// FIR    
    reg  [(pDATA_WIDTH-1):0] x;     // x[t]
    wire [(pDATA_WIDTH-1):0] x_tmp;
       
    reg  [(pDATA_WIDTH-1):0] h;     // h[i]
    wire [(pDATA_WIDTH-1):0] h_tmp;
    
    reg  [(pDATA_WIDTH-1):0] m;     // h[i] * x[t - i]
    wire [(pDATA_WIDTH-1):0] m_tmp;
    
    reg  [(pDATA_WIDTH-1):0] y;     // y[t] = sum of m
    wire [(pDATA_WIDTH-1):0] y_tmp;   
    
    assign h_tmp = tap_Do;        
    assign x_tmp = x_data;        
    assign m_tmp = h * x;
    assign y_tmp = (cnt_y == 4'd0)? m : m + y;
                                    // sum every 11 m  
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2]) begin
            h <= 32'd0;
            x <= 32'd0;
            m <= 32'd0;
            y <= 32'd0;
        end
        else begin
            h <= h_tmp;
            x <= x_tmp;
            m <= m_tmp;
            y <= y_tmp;
        end
    end
    
// axi_stream: sm finite state machine
    reg sm_state;
    reg sm_next_state;
    reg sm_tlast_tmp;
    
    always @* begin
        case (sm_state)
            `SM_IDLE:
            begin
                if (tlast_cnt_tmp == data_length) begin
                    sm_next_state = `SM_DONE;
                    sm_tlast_tmp = 1'b1;
                end
                else begin
                    sm_next_state = `SM_IDLE;
                    sm_tlast_tmp     = 1'b0;
                end
            end
            `SM_DONE:
            begin
                if (sm_tvalid == 1'b1) begin
                    sm_next_state = `SM_IDLE;
                    sm_tlast_tmp = 1'b0;
                end
                else begin
                    sm_next_state = `SM_DONE;
                    sm_tlast_tmp = 1'b0;
                end
            end
        endcase
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            sm_state <= `SM_DONE;
        else
            sm_state <= sm_next_state;
    end
    
// axi_stream:sm
    assign sm_tvalid = (cnt_y == 5'd0)? 1'b1 : 1'b0;    // pass y[t] to tb
    assign sm_tdata  = y;                               // pass y[t] to tb
    
    assign sm_tlast  = sm_tlast_tmp; 
endmodule