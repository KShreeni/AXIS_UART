module top_axis_uart #(parameter DATA_BITS = 8) (input clk,rst,input wire [7:0] axis_data,
input wire axis_valid,
input wire axis_last,output wire uart_tx,rx_valid,output m_axis_ready,output wire [DATA_BITS-1:0]rx_data);//,output wire parity_err);

wire [7:0] m_axis_data;
wire axis_ready;

wire m_axis_valid_out;

assign m_axis_ready = axis_ready;

axis_master_inp #(.WIDTH(8)) mast_inst(.clk(clk),.rst(rst),.load_data(axis_data),
.m_axis_valid(axis_valid),.m_axis_ready(axis_ready),.m_axis_last(axis_last),.m_axis_valid_out(m_axis_valid_out),.m_axis_data(m_axis_data));

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
    input  wire m_axis_valid,   // input from source (testbench)
    input wire m_axis_last,
    output reg  m_axis_valid_out,
    output reg  [WIDTH-1:0] m_axis_data
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            m_axis_data <= 0;
            m_axis_valid_out <= 0;
        end else begin
            // Hold valid high until handshake completes
          m_axis_valid_out <= m_axis_valid;
            // note: no "else" resetting valid_out

            // Update data only when handshake occurs
            if (m_axis_valid && m_axis_ready)
                m_axis_data <= load_data;
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
reg uart_valid_temp,s_axis_ready_temp;

assign s_axis_ready = !fifo_full;

always@(posedge clk or posedge rst)begin
if(rst)begin
 s_axis_ready_temp <= 0;
 
 end
else begin
 s_axis_ready_temp <= s_axis_ready;
 
 end
end

assign fifo_wr_en = s_axis_valid && s_axis_ready_temp;

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

module sync_fifo #(
    parameter WIDTH = 8,
    parameter DEPTH = 8
)(
    input  wire clk,
    input  wire rst,

    // Write interface
    input  wire wr_en,
    input  wire din_last,
    input  wire [WIDTH-1:0] din,
    output wire full,

    // Read interface
    input  wire rd_en,
    output reg  dout_last,
    output wire empty,
    output reg  [WIDTH-1:0] dout
);

    localparam ADDR_WIDTH = $clog2(DEPTH);

    reg [WIDTH-1:0] mem_data [0:DEPTH-1];
    reg             mem_last [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [ADDR_WIDTH:0]   count;

    integer i;

    // --------------------------
    // Sequential Logic
    // --------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
            dout   <= 0;
            dout_last <= 0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem_data[i] <= 0;
                mem_last[i] <= 0;
            end
        end 
        else begin
            // ===== WRITE =====
            if (wr_en && !full) begin
                mem_data[wr_ptr] <= din;
                mem_last[wr_ptr] <= din_last;
                wr_ptr <= wr_ptr + 1'b1;
            end

            // ===== READ =====
            if (rd_en && !empty) begin
                dout      <= mem_data[rd_ptr];
                dout_last <= mem_last[rd_ptr];
                rd_ptr    <= rd_ptr + 1'b1;
            end
            
            else if(dout_last == 1) dout_last <= 0;            

            // ===== COUNT UPDATE =====
            case ({wr_en && !full, rd_en && !empty})
                2'b10: count <= count + 1'b1;  // write only
                2'b01: count <= count - 1'b1;  // read only
                default: count <= count;       // both or none → no change
            endcase
        end
    end

    // --------------------------
    // Status Flags
    // --------------------------
    assign full  = (count == DEPTH);
    assign empty = (count == 0);

endmodule

module uart_tx #(
    parameter clk_rate = 50_000_000,
    parameter Baud     = 115200,
    parameter Word_len = 8,
    parameter PARITY   = "even" // Options: "none", "even", "odd"
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
    reg tx_data_ready_temp;
    reg tx_data_last_temp,tx_data_valid_temp;
    
    assign tx_data_ready = (next_state == Idle);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= Idle;
            tx_data_last_temp <= 0;
            tx_data_valid_temp <= 0;
            end
        else begin
            current_state <= next_state;
            tx_data_last_temp <= tx_data_last;
            tx_data_valid_temp <= tx_data_valid;
          end  
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            Idle: begin
                if (tx_data_valid && !tx_data_last_temp) next_state = Start;
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
                 if (tx_data_valid_temp) begin
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
                        shift_reg <= {1'b1, shift_reg[Word_len-1:1]}; // Shift in 1s
                        bit_cnt <= bit_cnt + 1;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end

                Parity: begin
                    Uart_tx <= parity_bit; // Send the calculated parity bit
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
    parameter PARITY      = "even"      // Options: "none", "even", "odd"
)(
    input  wire clk,
    input  wire rst,
    input  wire rx,                     // UART serial input line
    output reg  [DATA_BITS-1:0] rx_data, // Received byte
    output reg  rx_valid              // High for 1 clk when new data is ready and parity is OK
    //output reg  parity_error            // High for 1 clk if parity check fails
);

    localparam BAUD_DIV    = (CLK_FREQ / BAUD);
    localparam HALF_BAUD   = BAUD_DIV / 2;
    
    // FSM States
    localparam IDLE   = 3'd0,
               START  = 3'd1,
               DATA   = 3'd2,
               PARITY_S = 3'd3, // Parity State
               STOP   = 3'd4;
    
    reg [2:0] state, next_state;
    reg [$clog2(BAUD_DIV):0] baud_cnt;
    reg [$clog2(DATA_BITS):0] bit_cnt;
    reg [DATA_BITS-1:0] shift_reg;
    reg received_parity_bit;
    wire calculated_parity;
    wire parity_match;
   assign parity_match = (calculated_parity == received_parity_bit);

    // FSM state transition logic (sequential)
    assign calculated_parity = ^shift_reg;
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end

    // FSM next-state logic (combinational)
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (!rx) // Start bit detected
                    next_state = START;
            end
            START: begin
                if (baud_cnt == (BAUD_DIV-2))
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

    // FSM output and datapath logic (sequential)
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            baud_cnt <= 0;
            bit_cnt <= 0;
            shift_reg <= 0;
            rx_data <= 0;
            rx_valid <= 0;
           // parity_error <= 0;
            received_parity_bit <= 0;
//            parity_match <= 0;
        end else begin
            // Default assignments (de-assert pulses)
            rx_valid <= 0;
          //  parity_error <= 0;

            case (state)
                IDLE: begin
                    bit_cnt <= 0;
                    baud_cnt <= 0;
                end
                
                START: begin
                    if (baud_cnt == (BAUD_DIV-3)) begin
                       baud_cnt <= baud_cnt + 1;
                        bit_cnt <= 0;
                     end  
                     else  baud_cnt <= baud_cnt + 1;
                       
                       
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
                            rx_valid <= rx_valid; // *** PARITY CHECK LOGIC ***
                              //parity_error <= parity_error;
                            

//                            if (PARITY == "even")
//                                parity_match <= (calculated_parity == received_parity_bit);
//                            else // (PARITY == "odd")
//                                parity_match <= (calculated_parity != received_parity_bit);

                            if (parity_match) begin
                                rx_valid <= 1'b1; // Parity OK: Data is valid
                               // parity_error <= 1'b0;
                            end else begin
                                rx_valid <= 1'b0; // Parity FAILED: Data is invalid
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
