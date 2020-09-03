module CPU(
    input         clk,
    input         rst,
    output reg instr_read,
    output reg [31:0] instr_addr,
    input  [31:0] instr_out,
    output reg data_read,
    output reg data_write,
    output reg [31:0] data_addr,
    output reg [31:0] data_in,
    input  [31:0] data_out
);

    wire [31:0]data_addr_a;
    wire data_read_r;
    wire data_write_w;
    wire [31:0] data_in_i;
    wire [31:0] pc;

    always@(posedge clk) 
	begin
            if(rst)
	        begin
		    instr_read <= 1'b1;
                    instr_addr <= 32'd0;      
                end
	    else
		begin	    
		    instr_read <= ~instr_read;
		    instr_addr <= pc;   
		end
        end
    always@(*)
	begin
	    data_addr = data_addr_a;
    	    data_in = data_in_i;
    	    data_read = data_read_r;
    	    data_write = data_write_w;	
	end
    control e(.clk(clk),.instr(instr_out),.instr_addr(instr_addr),.data_read(data_read_r),.data_write(data_write_w),.pc(pc),.data_addr(data_addr_a),.data_in(data_in_i),.data_out(data_out)); 
   endmodule
module control(
    input clk,
    input [31:0] instr,
    input [31:0] instr_addr,
    output wire data_read,
    output wire data_write,
    output wire [31:0] pc,
    output wire [31:0] data_addr,
    output wire [31:0] data_in,
    input  [31:0] data_out
);
    reg rd_write;
    wire [31:0] rd_in;
    wire [31:0] rs;
    wire [31:0] rt;
    wire [31:0] out;
    always@(*) begin
        if(instr[6:0] == 7'b0000011 ||instr[6:0]==7'b0110111 || instr[6:0]==7'b0010111 || instr[6:0]==7'b1101111 || instr[6:0]==7'b1100111 || instr[6:0]==7'b0010011 || instr[6:0]==7'b0110011)
            rd_write = 1'b1;
        else
            rd_write = 1'b0;
	end
    get_data b(.clk(clk),.rd_write(rd_write),.instr(instr),.rd_in(rd_in),.rs(rs),.rt(rt));
    ALU c(.instr(instr),.rs(rs),.rt(rt),.out(out));
    data_and_pc d(.clk(clk),.instr(instr),.instr_addr(instr_addr),.out(out),.rs(rs),.rt(rt),.data_out(data_out),.pc(pc),.data_in(data_in),.data_addr(data_addr),.data_read(data_read),.data_write(data_write),.rd_in(rd_in));
endmodule

module get_data(
    input clk,
    input rd_write,
    input [31:0] instr,
    input [31:0] rd_in,
    output reg [31:0] rs,
    output reg [31:0] rt
);
    reg [31:0] registers[0:31];
    integer i;
    initial begin
        for(i = 0;i < 32;i = i + 1)
            registers[i] = 32'd0;
        end
    always@(posedge clk) 
	begin
            if(rd_write==1'b1 && (instr[11:7]!=5'd0))
                registers[instr[11:7]]<= rd_in;
	    else
	        registers[instr[11:7]]<=registers[instr[11:7]];	
        end
    always@(*)
	begin
	    case(instr[6:0])    
	    7'b0000011:begin
		rs = registers[instr[19:15]];
		rt = 32'd0;
		end
	    7'b0010011:begin
		rs = registers[instr[19:15]];
		rt = 32'd0;
		end
	    7'b1100111:begin
		rs = registers[instr[19:15]];
		rt = 32'd0;
		end
	    7'b0010111:begin
		rs = 32'd0;
		rt = 32'd0;
		end
	    7'b0110111:begin
		rs = 32'd0;
		rt = 32'd0;
		end
	    7'b1101111:begin
		rs = 32'd0;
		rt = 32'd0;
		end
	   default:begin
		rs = registers[instr[19:15]];
		rt = registers[instr[24:20]];
		end
	    endcase
	end
endmodule

module ALU(
    input [31:0] instr,
    input [31:0] rs,
    input [31:0] rt,
    output reg [31:0] out
);
    always@(*)
	begin
	    case(instr[6:0]) 
	    7'b0110011:begin
		case(instr[14:12])
		3'b000:begin
		    case(instr[31:25])
		    7'b0000000:out = rs + rt;
		    7'b0100000:out = rs - rt;
		    default: out = 32'd0;
		    endcase
		    end
		3'b001:out = $unsigned(rs) << rt[4:0];
		3'b010:out = ($signed(rs) < $signed(rt))?32'd1:32'd0;
		3'b011:out = ($unsigned(rs) < $unsigned(rt))?32'd1:32'd0;
		3'b100:out = rs ^ rt;
		3'b101:begin
		    case(instr[31:25])
		    7'b0000000:out = $unsigned(rs) >> rt[4:0];
		    7'b0100000:out = $signed(rs) >>> rt[4:0];
		    default: out = 32'd0;
		    endcase
		    end
		3'b110:out = rs | rt;
		3'b111:out = rs & rt;
		default:out = 32'd0;
		endcase
		end  
	    7'b0010011:begin
		case(instr[14:12])
		3'b000:out = rs + {{20{instr[31]}},instr[31:20]};
		3'b001:out = $unsigned(rs) << instr[24:20];
		3'b010:out = ($signed(rs) < $signed({{20{instr[31]}},instr[31:20]}))?32'd1:32'd0;
		3'b011:out = ($unsigned(rs) < $unsigned({{20{instr[31]}},instr[31:20]}))?32'd1:32'd0;
		3'b100:out = rs ^ {{20{instr[31]}},instr[31:20]};
		3'b101:begin
		    case(instr[31:25])
		    7'b0000000:out = $unsigned(rs) >> instr[24:20];
		    7'b0100000:out = $signed(rs) >>> instr[24:20];
		    default: out = 32'd0;
		    endcase
		    end
		3'b110:out = rs | {{20{instr[31]}},instr[31:20]};
		3'b111:out = rs & {{20{instr[31]}},instr[31:20]};
		default:out = 32'd0;
		endcase
		end
	    7'b1100111:out = 32'd0; //JALR
	    7'b0100011:out = 32'd0; //sw
	    7'b1100011:begin
		case(instr[14:12])
		3'b000:out = (rs == rt)?32'd1:32'd0;
		3'b001:out = ( rs != rt )?32'd1:32'd0;
		3'b100:out = ($signed(rs) < $signed(rt))?32'd1:32'd0;
		3'b101:out = ($signed(rs) >= $signed(rt))?32'd1:32'd0;
		3'b110:out = ($unsigned(rs) < $unsigned(rt))?32'd1:32'd0;
		3'b111:out = ($unsigned(rs) >= $unsigned(rt))?32'd1:32'd0;
		default:out = 32'd0;
		endcase
		end
	    default:out = 32'd0;
	    endcase
	end
endmodule

module data_and_pc(
    input clk,
    input [31:0] instr,
    input [31:0] instr_addr,
    input [31:0] out,
    input [31:0] rs,
    input [31:0] rt,
    input [31:0]data_out,
    output reg [31:0] pc,
    output reg [31:0] data_in,
    output reg [31:0] data_addr,
    output reg data_read,
    output reg data_write,
    output reg [31:0] rd_in
);
    always@(*)
	begin
	    case(instr[6:0])
	    7'b1100111:begin
		data_in = 32'd0;
		data_addr = 32'd0;
		data_read = 1'b0;
		data_write = 1'b0;
		rd_in = instr_addr + 32'd4;
		pc = rs + {{20{instr[31]}},instr[31:20]};
		end
	    7'b0000011:begin
		data_in = 32'd0;
		data_addr = rs + {{20{instr[31]}},instr[31:20]};
		data_read = 1'b1;
		data_write = 1'b0;
		rd_in = data_out;
		pc = instr_addr + 32'd4;
		end
	    7'b0100011:begin
		data_in = rt;
		data_addr = rs + {{20{instr[31]}},instr[31:25],instr[11:7]};
		rd_in = 32'd0;
		data_read = 1'b0;
		data_write = 1'b1;
		pc = instr_addr + 32'd4;
		end
	    7'b1100011:begin
		data_in = 32'd0;
		data_read = 1'b0;
		data_write = 1'b0;
		rd_in = 32'd0;
		if(out == 32'd1)
		    pc = instr_addr + {{19{instr[31]}},instr[31],instr[7],instr[30:25],instr[11:8],1'b0};
		else
		    pc = instr_addr + 32'd4;
		end
	    7'b0010111:begin
		data_read = 1'b0;
		rd_in = instr_addr + {instr[31:12],12'd0};
		data_in = 32'd0;
		data_addr = 32'd0;
		data_write = 1'b0;
		pc = instr_addr + 32'd4;
		end
	    7'b0110111:begin
		rd_in = {instr[31:12],12'd0};
		data_in = 32'd0;
		data_read = 1'b0;
		data_addr = 32'd0;
		data_write = 1'b0;
		pc = instr_addr + 32'd4;
		end
	    7'b1101111:begin
		rd_in = instr_addr + 32'd4;
		data_in = 32'd0;
		data_addr = 32'd0;
		data_read = 1'b0;
		data_write = 1'b0;
		pc = instr_addr + {{11{instr[31]}},instr[31],instr[19:12],instr[20],instr[30:21],1'b0};
		end
	    default:begin
		data_in = 32'd0;
		rd_in = out;
		data_addr = 32'd0;
		data_read = 1'b0;
		data_write = 1'b0;
		pc = instr_addr + 32'd4;
		end
	    endcase
	end
endmodule