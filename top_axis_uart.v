module top_axis_uart #(parameter DATA_BITS = 8) (input clk,rst,input wire [7:0] axis_data,
input wire axis_valid,
input wire axis_last,output wire uart_tx,rx_valid,output m_axis_ready,output wire [DATA_BITS-1:0]rx_data);//,output wire parity_err);

wire [7:0] m_axis_data;
wire axis_ready;

wire m_axis_valid_out;

assign m_axis_ready = axis_ready;

axis_master_inp #(.WIDTH(8)) mast_inst(.clk(clk),.rst(rst),.load_data(axis_data),
.m_axis_valid(axis_valid),.m_axis_ready(axis_ready),.m_axis_valid_out(m_axis_valid_out),.m_axis_data(m_axis_data));

axis_fifo_uart_tx #(.WIDTH(8),.DEPTH(8),.CLK_RATE(50000000),.BAUD(115200)) axis_fifo_uart_tx_inst(.clk(clk),.rst(rst),
.s_axis_data(m_axis_data),.s_axis_valid(m_axis_valid_out),.s_axis_ready(axis_ready),.s_axis_last(axis_last),.uart_tx(uart_tx));

uart_rec #(.CLK_FREQ(50000000),.BAUD(115200),.DATA_BITS(DATA_BITS)) 
uart_rec_inst(.clk(clk),.rst(rst),.rx(uart_tx),.rx_data(rx_data),.rx_valid(rx_valid));//.parity_error(parity_err));
endmodule

module axis_master_inp #(parameter WIDTH = 8) (
    input  wire clk,
    input  wire rst,  
    input  wire [WIDTH-1:0] load_data,

    input  wire m_axis_ready,
    input  wire m_axis_valid,
   
     output reg m_axis_valid_out,
    output reg  [WIDTH-1:0] m_axis_data             // output data
);
   
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            m_axis_data <= 0;
            m_axis_valid_out <= 0;
        end
        else begin
           if(m_axis_valid && m_axis_ready) begin
                m_axis_valid_out <= 1;
                 m_axis_data <= load_data;
           end
           else begin
              m_axis_valid_out <= 0;
               m_axis_data <=  m_axis_data;

          end

       end
end
endmodule

module axis_fifo_uart_tx #(parameter WIDTH = 8, DEPTH = 8, CLK_RATE = 50000000, BAUD = 115200)
(
 input clk,rst,
 input [WIDTH-1:0] s_axis_data,
 input s_axis_valid,
 input s_axis_last,
 output wire s_axis_ready,
 output wire uart_tx
);

wire[WIDTH-1:0] fifo_out;
wire fifo_out_last;
wire fifo_full,fifo_empty;
wire fifo_wr_en,fifo_rd_en;
reg uart_valid_temp;

assign s_axis_ready = !fifo_full;
assign fifo_wr_en = s_axis_valid && s_axis_ready;

always@(posedge clk or posedge rst)begin
if(rst)
 uart_valid_temp <= 0;
else 
 uart_valid_temp <= !fifo_empty;
 end
wire uart_valid = uart_valid_temp;
wire uart_ready;

assign fifo_rd_en = uart_valid && uart_ready;

sync_fifo #(.WIDTH(WIDTH),.DEPTH(DEPTH)) fifo_inst(.clk(clk),.rst(rst),
.wr_en(fifo_wr_en),.din(s_axis_data),.din_last(s_axis_last),.full(fifo_full),
.empty(fifo_empty),.rd_en(fifo_rd_en),.dout(fifo_out),.dout_last(fifo_out_last));

uart_tx #(.clk_rate(CLK_RATE),.Baud(BAUD),.Word_len(WIDTH)) uart_inst(.clk(clk),.rst(rst),
.tx_data(fifo_out),.tx_data_valid(uart_valid),.tx_data_ready(uart_ready),.tx_data_last(fifo_out_last),
.Uart_tx(uart_tx));


endmodule    


module sync_fifo #(parameter WIDTH=8,DEPTH=8)
(
input clk,rst,
input wr_en,din_last,
input[WIDTH-1:0] din,
output wire full,
input rd_en,
output reg dout_last,
output wire empty,
output reg[WIDTH-1:0] dout
);

localparam ADDR_width = $clog2(DEPTH);

reg [WIDTH-1:0] mem_data [0:DEPTH-1];
reg    mem_last [0:DEPTH-1];

reg [ADDR_width-1:0] wr_ptr,rd_ptr;
reg [ADDR_width:0] count;

integer i;

always@(posedge clk or posedge rst)begin
 if(rst)begin
   wr_ptr <= 0;
   rd_ptr <= 0;
   count <= 0;
   for(i=0;i<DEPTH;i=i+1)begin
     mem_data[i] <= {WIDTH{1'b0}};
     mem_last[i] <= 1'b0;
   end
   dout <= {WIDTH{1'b0}};
   dout_last <= 1'b0;
  end
  else begin
    if(wr_en && !full)begin
     mem_data[wr_ptr] <= din;
     mem_last[wr_ptr] <= din_last;
     wr_ptr <= wr_ptr + 1;
     count <= count + 1;
   end
   if(rd_en && !empty)begin  
     dout <= mem_data[rd_ptr];
     dout_last <= mem_last[rd_ptr];
     rd_ptr <= rd_ptr + 1;
     count <= count - 1;
   end
   else begin
     dout <= dout;
     dout_last <= dout_last;
   end
 end
end

assign full = (count == DEPTH);
assign empty = (count == 0);

endmodule

module uart_tx #(
    parameter clk_rate = 50_000_000,
    parameter Baud     = 115200,
    parameter Word_len = 8,
    parameter PARITY   = "even" 
)
(
    input clk, rst,
    input [Word_len-1:0] tx_data,
    input tx_data_valid,tx_data_last,
    output wire tx_data_ready,
    output reg Uart_tx
);

    localparam Baud_div = (clk_rate / Baud);
    
    // FSM States
    localparam Idle   = 3'd0,
               Start  = 3'd1,
               Data   = 3'd2,
               Parity = 3'd3,
               Stop   = 3'd4;

    reg [2:0] current_state, next_state;

    reg [$clog2(Baud_div)-1:0] baud_cnt;
    reg [$clog2(Word_len)-1:0] bit_cnt;
    reg [Word_len-1:0] shift_reg;
    reg parity_bit;

    assign tx_data_ready = (current_state == Idle);

    always @(posedge clk or posedge rst) begin
        if (rst)
            current_state <= Idle;
        else
            current_state <= next_state;
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            Idle: begin
                if (tx_data_valid && !tx_data_last) next_state = Start;
                else next_state = Idle;
            end
            Start: begin
                if (baud_cnt == (Baud_div - 1)) next_state = Data;
            end
            Data: begin
                if (bit_cnt == Word_len - 1 && baud_cnt == (Baud_div - 1)) begin
                    if (PARITY == "none")
                        next_state = Stop;
                    else
                        next_state = Parity;
                end
            end
            Parity: begin
                if (baud_cnt == (Baud_div - 1)) next_state = Stop;
            end
            Stop: begin
                if(baud_cnt == (Baud_div - 1)) next_state = Idle;
                 else if (tx_data_last)next_state = Idle;
                  else next_state = Stop;
            end
            default: next_state = Idle;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            baud_cnt <= 0;
            bit_cnt <= 0;
            shift_reg <= 0;
            Uart_tx <= 1'b1;
            parity_bit <= 1'b0;
        end else begin
             case (current_state)
                Idle: begin
                    baud_cnt <= 0;
                    bit_cnt <= 0;
                    Uart_tx <= 1'b1;
                   
                end
                
                Start: begin
                 if (tx_data_valid) begin
                        shift_reg <= tx_data;
                        // Pre-calculate the parity bit
                        if (PARITY == "even")
                            parity_bit <= ^tx_data;
                        else if (PARITY == "odd")
                            parity_bit <= ~^tx_data;
                    end
                    Uart_tx <= 1'b0;
                    if (baud_cnt == (Baud_div - 1))
                        baud_cnt <= 0;
                    else
                        baud_cnt <= baud_cnt + 1;
                end
                
                Data: begin
                    Uart_tx <= shift_reg[0];
                    if (baud_cnt == (Baud_div - 1)) begin
                        baud_cnt <= 0;
                        shift_reg <= {1'b1, shift_reg[Word_len-1:1]}; 
                        bit_cnt <= bit_cnt + 1;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end

                Parity: begin
                    Uart_tx <= parity_bit; 
                     if (baud_cnt == (Baud_div - 1))
                        baud_cnt <= 0;
                    else
                        baud_cnt <= baud_cnt + 1;
                end
                
                Stop: begin
                    Uart_tx <= 1'b1;
                    if (baud_cnt == (Baud_div - 1))
                        baud_cnt <= 0;
                    else
                        baud_cnt <= baud_cnt + 1;
                end
            endcase
        end
    end
endmodule

module uart_rec #(
    parameter CLK_FREQ    = 50_000_000,
    parameter BAUD        = 115200,
    parameter DATA_BITS   = 8,
    parameter PARITY      = "even"      
)(
    input  wire clk,
    input  wire rst,
    input  wire rx,                    
    output reg  [DATA_BITS-1:0] rx_data, 
    output reg  rx_valid              
    //output reg  parity_error            
);

    localparam BAUD_DIV    = (CLK_FREQ / BAUD);
    localparam HALF_BAUD   = BAUD_DIV / 2;
    

    localparam IDLE   = 3'd0,
               START  = 3'd1,
               DATA   = 3'd2,
               PARITY_S = 3'd3, 
               STOP   = 3'd4;
    
    reg [2:0] state, next_state;
    reg [$clog2(BAUD_DIV):0] baud_cnt;
    reg [$clog2(DATA_BITS):0] bit_cnt;
    reg [DATA_BITS-1:0] shift_reg;
    reg received_parity_bit;
    wire calculated_parity;
    reg parity_match;  
    
    assign calculated_parity = ^shift_reg;
    
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end
    
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (!rx) // Start bit detected
                    next_state = START;
            end
            START: begin
                if (baud_cnt == HALF_BAUD)
                    next_state = DATA;
            end
            DATA: begin
                if (baud_cnt == (BAUD_DIV - 1) && bit_cnt == DATA_BITS - 1) begin
                    if (PARITY == "none")
                        next_state = STOP;
                    else
                        next_state = PARITY_S;
                end
            end
            PARITY_S: begin
                if (baud_cnt == (BAUD_DIV - 1))
                    next_state = STOP;
                  //  calculated_parity = ^shift_reg;
            end
            STOP: begin
                if (baud_cnt == (BAUD_DIV - 1))
                    next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

   
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            baud_cnt <= 0;
            bit_cnt <= 0;
            shift_reg <= 0;
            rx_data <= 0;
            rx_valid <= 0;
           // parity_error <= 0;
            received_parity_bit <= 0;
            parity_match <= 0;
        end else begin
          
            rx_valid <= 0;
          //  parity_error <= 0;

            case (state)
                IDLE: begin
                    bit_cnt <= 0;
                    baud_cnt <= 0;
                end
                
                START: begin
                    if (baud_cnt == BAUD_DIV/2) begin
                        baud_cnt <= 0;
                        bit_cnt <= 0;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end
                
                DATA: begin
                    if (baud_cnt == (BAUD_DIV - 1)) begin
                        baud_cnt <= 0;
                        shift_reg <= {rx, shift_reg[DATA_BITS-1:1]};
                        bit_cnt <= bit_cnt + 1;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end

                PARITY_S: begin
                    if (baud_cnt == (BAUD_DIV - 1)) begin
                        baud_cnt <= 0;
                        received_parity_bit <= rx; // Capture the parity bit
                    end else
                        baud_cnt <= baud_cnt + 1;
                end
                
                STOP: begin
                    if (baud_cnt == (BAUD_DIV - 1)) begin
                        baud_cnt <= 0;
                        rx_data <= shift_reg;

                        if (PARITY == "none") begin
                            rx_valid <= 1'b1;
                           // parity_error <= 1'b0;
                        end else begin
                            rx_valid <= rx_valid; 
                              //parity_error <= parity_error;
                            

                            if (PARITY == "even")
                                parity_match <= (calculated_parity == received_parity_bit);
                            else 
                                parity_match <= 0;//(calculated_parity != received_parity_bit);

                            if (parity_match) begin
                                rx_valid <= 1'b1; 
                               // parity_error <= 1'b0;
                            end else begin
                                rx_valid <= 1'b0; 
                                //parity_error <= 1'b1;
                            end
                        end
                    end else
                        baud_cnt <= baud_cnt + 1;
                end
            endcase
        end
    end
                          
endmodule
