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
