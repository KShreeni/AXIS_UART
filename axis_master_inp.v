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
