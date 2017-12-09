////////////////////////START ALU MODULE/////////////////////////
module Alu (a,b,alu_ctrl_ip,shamt,result,zero);

input wire signed[31:0] a;  //input 1 

input wire signed [31:0] b; // input 2

input wire [4:0] shamt; // shift amount input

input wire  [2:0] alu_ctrl_ip; //the operation

output  wire signed [31:0] result;  // the result

output reg zero; 

assign result = (alu_ctrl_ip==3'b010)? (a+b): //add

(alu_ctrl_ip==3'b110)? (a-b)  : //subtract
(alu_ctrl_ip==3'b000) ? (a&b) : // and
(alu_ctrl_ip==3'b001) ? (a|b) : // or
(alu_ctrl_ip==3'b100) ? (b<<shamt):  // shift left logically

32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx; // the default value otherwise
always @(result)
begin
if (result==0)
begin
zero=1'b1; 
end
else 
begin
zero=1'b0;
end
end

endmodule
////////////////////////END ALU MODULE/////////////////////////


////////////////////////START ALU_CTRL MODULE AND TB/////////////////////////
module Alu_Control (Funct,ALUOp,ALUOperation);
   input [1:0] ALUOp;
    input [5:0] Funct;
    output [2:0] ALUOperation;
    reg    [2:0] ALUOperation;
    // symbolic constants for instruction function code
    parameter F_add = 6'd32;
    parameter F_sub = 6'd34;
    parameter F_and = 6'd36;
    parameter F_or  = 6'd37;
    parameter F_slt = 6'd42;
    parameter F_sll = 6'd0;
    // symbolic constants for ALU Operations
    parameter ALU_add = 3'b010;
    parameter ALU_sub = 3'b110;
    parameter ALU_and = 3'b000;
    parameter ALU_or  = 3'b001;
    parameter ALU_slt = 3'b111;
    parameter ALU_sll = 3'b100;
    always @(ALUOp or Funct)
    begin
        case (ALUOp) 
            2'b00 : ALUOperation = ALU_add;
            2'b01 : ALUOperation = ALU_sub;
            2'b10 : case (Funct)                          //R_Format
                        F_add : ALUOperation = ALU_add;
                        F_sub : ALUOperation = ALU_sub;
                        F_and : ALUOperation = ALU_and;
                        F_or  : ALUOperation = ALU_or;
                        F_slt : ALUOperation = ALU_slt;
			F_sll : ALUOperation = ALU_sll;
                        default ALUOperation = 3'bxxx;
                    endcase
            default ALUOperation = 3'bxxx;
        endcase
    end
endmodule 
module testb();

reg [6:0] func;
reg [1:0] alu_op;
wire [3:0] alu_ctrl_ip;

Alu_Control myctrl (func,alu_op,alu_ctrl_ip);

initial

begin

func=6'b100000;
alu_op=2'b00;
$strobe ("the alu ctrl_ip is %b which perform addition in case lw and sw  ", alu_ctrl_ip);
#5
func=6'b100010;
alu_op=2'b01;
$strobe ("the alu ctrl_ip is %b which perform subtraction in case beq ", alu_ctrl_ip);

#5
func=6'b100000;
alu_op=2'b10;
$strobe ("the alu ctrl_ip is %b which perform addition  ", alu_ctrl_ip);
#5
func=6'b100010;
alu_op=2'b10;
$strobe ("the alu ctrl_ip is %b which perform subtraction  ", alu_ctrl_ip);
#5
func=6'b100100;
alu_op=2'b10;
$strobe ("the alu ctrl_ip is %b which perform and  ", alu_ctrl_ip);
#5
func=6'b100101;
alu_op=2'b10;
$strobe ("the alu ctrl_ip is %b which perform or  ", alu_ctrl_ip);
#5
func=6'b000000;
alu_op=2'b10;
$strobe ("the alu ctrl_ip is %b which perform sll  ", alu_ctrl_ip);
#5
func=6'b101010;
alu_op=2'b10;
$strobe ("the alu ctrl_ip is %b which perform slt  ", alu_ctrl_ip);

end
endmodule
 

////////////////////////END ALU_CTRL MODULE AND TB/////////////////////////

////////////////////////START REG_FILE MODULE/////////////////////////
  

module Reg_File(input clk,[4:0] RR1,[4:0] RR2, [4:0] WR_reg,wire [31:0] WR_data,wire write_enable,      
output  reg[31:0]D1, reg [31:0] D2);
reg [31:0] A [31:0];  
initial
begin
A[8]=16;//$t0 
A[9]=16; // $t1 
A[10]=18; // $t2
A[11]=19; // $t3
A[12]=20;  //$t4
A[16]=21; //$s0
A[17]=22; //$s1
A[18]=23;//$s2
A[19]=24;//$s3
A[20]=25; //$s4

//$monitor($time,,,"%d",A[5]);
end  
always @(posedge clk or RR1 or RR2)     
begin
if(write_enable)
begin
A[WR_reg]<=WR_data; 
D1<=A[RR1];
D2<=A[RR2]; 
 
end
else
begin
D1=A[RR1];
D2=A[RR2];
end
end
endmodule

////////////////////////END REG_FILE MODULE/////////////////////////

module Reg_Sign_Ext(input [4:0] reg_addr,output wire[31:0]result);

assign result [4:0]=reg_addr[4:0];
assign result [31:5]=$signed (reg_addr[4]);


endmodule


////////////////////////START MUX MODULE/////////////////////////


module MUX (input [31:0]WD,input [31:0]result,input MUX_ctrl,output reg [31:0] z );
initial
begin
z=0;
end
always @(WD,result,MUX_ctrl)
begin
assign z=(MUX_ctrl==0)? WD : result ;
//if(MUX_ctrl==0)
//z=WD;
//else if(MUX_ctrl==1)
//z=result;
//else
//z=0;
end
endmodule


module tb_tb_tb_Mux();
reg [31:0]x;
reg [31:0]y;
reg sel;
wire [31:0]ooo1;

initial
begin
y=10;
sel=1;


end
MUX mmm(x,y,sel, ooo1 );
endmodule








////////////////////////END MUX/////////////////////////


////////////////////////START CONTROL UNIT MODULE AND TB/////////////////////////


module Control( input[5:0] opcode,output reg [2:0]inst_type,output reg reg_dst,reg alu_src, reg mem_to_reg,reg reg_write,
reg mem_read,reg mem_write,reg branch,reg[1:0]alu_op, jump);
always@(opcode)
begin
case(opcode)

  6'b000000:          //R_Format
begin     
		inst_type=0;
		reg_dst = 1;
		alu_src = 0;
                mem_to_reg = 0;
		reg_write = 1; 
		mem_read = 0;
	        mem_write = 0;
	        branch = 0;
                alu_op = 2'b10;  
                jump = 0; 
end
////
  6'b100011:          //lw
begin     
		inst_type=1;
		reg_dst = 0;
		alu_src = 1;
                mem_to_reg = 1;
		reg_write = 1; 
		mem_read = 1;
	        mem_write = 0;
	        branch = 0;
                alu_op = 2'b00;  
                jump = 0; 
end
////
 6'b101011:          //sw
begin     
		inst_type=2;
		reg_dst = 1'bx;
		alu_src = 1;
                mem_to_reg = 1'bx;
		reg_write = 0; 
		mem_read = 0;
	        mem_write = 1;
	        branch = 0;
                alu_op = 2'b00;  
                jump = 0; 
end
////
 6'b000100:          //beq
begin   
		inst_type=3;  
		reg_dst = 1'bx;
		alu_src = 0;
                mem_to_reg = 1'bx;
		reg_write = 0; 
		mem_read = 0;
	        mem_write = 0;
	        branch = 1;
                alu_op = 2'b01;  
                jump = 0; 
end
////
 6'b000010:          //J
begin 
		inst_type=4;    
		reg_dst = 1'bx;
		alu_src = 1'bx;
                mem_to_reg = 1'bx;
		reg_write = 0; 
		mem_read = 0;
	        mem_write = 0;
	        branch = 0;
                alu_op = 2'bxx;  
                jump = 1; 
end
////
default:  begin   
		inst_type=3'bxxx;    
		reg_dst = 1'bx;
		alu_src = 1'bx;
                mem_to_reg = 1'bx;
		reg_write = 0; 
		mem_read = 0;
	        mem_write = 0;
	        branch = 0;
                alu_op = 2'bxx;  
                jump = 1; 
end    
endcase
end
endmodule




module TB_Control();
reg [5:0]opcode;
wire reg_dst,alu_src,mem_to_reg,reg_write,mem_read,mem_write,branch,jump;
wire[1:0]alu_op;
initial
begin

$monitor("%b %b %b %b %b %b %b %b %b %b",opcode,reg_dst,alu_src,mem_to_reg,reg_write,mem_read,mem_write,branch,alu_op,jump);
#5
opcode= 6'b000000;
#5
opcode= 6'b100011;
#5
opcode= 6'b101011;
#5
opcode= 6'b000100;
#5
opcode= 6'b000010;
#5
opcode=6'b111100;
end
Control c1(opcode,reg_dst,alu_src,mem_to_reg,reg_write,
mem_read,mem_write,branch,alu_op,jump);
endmodule


////////////////////////END CONTROL UNIT MODULE AND TB/////////////////////////


////////////////////////START PC MODULE AND TB/////////////////////////

module PC(input [31:0] new_address,input clk,input wire data_hz_ctrl,output reg [31:0] current_address);
initial 
begin
current_address=0;
end
always@ (posedge clk)
if(data_hz_ctrl==0)
begin 
current_address<=current_address;
end
else
begin
 current_address <= new_address;
end
endmodule 




module test_PC();
reg [31:0]new_address;
reg Clk=0;
wire [31:0]current_address;

initial
begin
$monitor("new= %b and current= %b",new_address,current_address);
#5
new_address=32'b1000_1011_1111_1000_1011_1101_1011_0001;
#5
new_address=32'b1011_1011_1111_1000_1010_1110_1011_0001;
end

always #5 Clk=~Clk;
PC T1(new_address,Clk,current_address);
endmodule

////////////////////////END PC MODULE AND TB/////////////////////////



////////////////////////START ShiftLeftBy2 MODULE AND TB/////////////////////////



module ShiftLeftBy2(input [31:0]signExtend,output reg [31:0] shifted);
always@*
begin
shifted <= signExtend << 0 ;	
end
endmodule




module SLL_Test();
reg [31:0]signExtend;
reg Clk=0;
wire [31:0]shifted;
initial
begin
$monitor("%b >> %b",signExtend,shifted);
#5
signExtend=32'b1000_1011_1111_1000_1011_1101_1011_0001;
#10
signExtend=32'b1011_1011_1111_1000_1010_1110_1011_0001; 
end

always #5 Clk=~Clk;
ShiftLeftBy2 T2(signExtend,shifted);
endmodule

////////////////////////START ShiftLeftBy2 MODULE AND TB/////////////////////////



////////////////////////START ADDER MODULE AND TB/////////////////////////



module Adder(input [31:0] first, input [31:0]second, output reg[31:0] result);
initial 
begin
result=0;
end
always@(first or second)
result <= first + second;
endmodule




module Add_Test();
reg [31:0] first;
reg [31:0] second;
reg Clk;
wire [31:0] result;

initial
begin
$monitor("%b + %b = %b",first,second,result);
#5
first=32'b1000_1011_1111_1000_1011_1101_1011_0001;
second=32'b1011_1011_1111_1000_1010_1110_1011_0001;
#10
second=32'b1000_1011_1111_1000_1011_1101_1011_0001;
end

always #5 Clk=~Clk;
Adder T3(first,second,result);
endmodule


////////////////////////END ADDER MODULE AND TB/////////////////////////



////////////////////////START AND MODULE AND TB/////////////////////////


module And(input zero, input branch, output reg PCSrc);
initial
begin
PCSrc=0;
end
 always@(zero or branch)
begin
PCSrc=zero&branch;
end
endmodule




module testAND();

reg zero,branch;
wire PCSrc;

initial
begin
$monitor($time,,"%b",PCSrc);
zero=0;
branch=0;
#5
zero=1;
#5
branch=1;
#5
branch=0;
end

And T4(zero,branch,PCSrc);
endmodule

////////////////////////END AND MODULE AND TB/////////////////////////



////////////////////////START Sign Extension MODULE AND TB/////////////////////////


module Sign_Extension (unext,result);

input wire [15:0] unext;
output wire [31:0] result;

assign result [15:0]=unext[15:0];
assign result [31:16]=$signed (unext[15]);

endmodule 





module tb_sign ();

reg [15:0] unext;
wire [31:0]result;

Sign_Extension extend (unext,result);

initial 
begin
unext = 16'b1000010000000000;

$strobe ("extended is %b",result);

end
endmodule



////////////////////////START Sign Extension MODULE AND TB/////////////////////////



////////////////////////START DATA MEMORY MODULE AND TB/////////////////////////


module Data_Memory (
                input [31:0] addr ,
                input [31:0] int_data ,
		input  mem_write ,mem_read, clk ,
		output reg [31:0] out_data 
		) ; 

reg [31:0] Data_Memory [63:0] ; 

initial
begin
$readmemb("binary.txt",Data_Memory);         
Data_Memory[48]=10;
Data_Memory[44]=14;
Data_Memory[46]=15;
Data_Memory[52]=15;
//$display("%b",Data_Memory[0]);
end

always@(posedge clk) 
	begin 
	if ( mem_write==1 )  
	begin    
	Data_Memory[addr] = int_data ; 
	//out_data=Data_Memory[addr];//this just for testing in top module
	end
	else if ( mem_read==1)          
	out_data = Data_Memory [addr] ;
	end

endmodule 

//testing Data_memory:


module Tb_Data_Memory;
reg [31:0] addr;
reg mem_write;
reg mem_read;
reg [31:0] int_data ; 
reg clk;
wire [31:0] out_data ; 


always 
#50 clk <= ~clk ;      //create clk


initial
begin 
clk=0;
int_data = 1 ; 



#50
addr= 0 ; 
mem_write= 0 ; 
mem_read= 1 ;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;

#100
addr= 1 ; 
mem_write= 0 ; 
mem_read= 1 ;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;

#100
addr= 2 ; 
mem_write= 0 ; 
mem_read= 1 ;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;

#100
addr= 3 ; 
mem_write= 0 ; 
mem_read= 1 ;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;

#100
addr= 5 ;
int_data=10;
mem_write= 1 ; 
mem_read= 0 ;

#100 
addr=5;
mem_write= 0 ; 
mem_read= 1 ;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;



end
Data_Memory My_Data_Memory (addr,int_data ,mem_write ,mem_read,clk ,out_data) ; 

endmodule 


////////////////////////END DATA MEMORY MODULE AND TB/////////////////////////
module Instruction_Memory (
                input [31:0] addr ,
		input  mem_write ,mem_read, clk ,
		output reg [31:0] out_data 
		) ; 

reg [31:0] Instruction_Memory [63:0] ; 	   //costruct the memory

//initial       		 //for test only ^^
//begin
//Instruction_Memory [0]=0;
//Instruction_Memory [1]=1;
//Instruction_Memory [2]=2;
//Instruction_Memory [3]=3;
//Instruction_Memory [4]=4;
//end

initial 
begin
$readmemb("binary.txt",Instruction_Memory);
//out_data = Instruction_Memory  [1] ; 
$display("%b",Instruction_Memory[0]);
end

always@(posedge clk or addr) 
	begin 
	if ( mem_read == 1 )
	begin      
	out_data = Instruction_Memory[addr];
	end

	end 
endmodule 

module Tb_Instruction_Memory;
reg [31:0] addr;
reg mem_write;
reg mem_read;
reg clk;
wire [31:0] out_data ; 


always 
#50 clk <= ~clk ;      //create clk
initial
begin 
clk=0;
addr= 0; 
mem_write= 0 ; 
mem_read= 1 ; 


#50
addr= 0 ; 
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;


#100
addr= 1 ; 
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;


#100
addr= 2 ; 
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;


#100
addr=3;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;


#100
addr=4;
$strobe ($time,,,"%d %d %b",clk,addr,out_data) ;



end
Instruction_Memory My_Instruction_Memory (addr,mem_write ,mem_read,clk ,out_data) ; 

endmodule 
////////////////////////START TOP MODULE AND TB/////////////////////////
//_________________DECLARING VARIABLES IN TOP MODULE_________________
/*
module Top_Mips_Processor(input clk,input mem_read,input mem_write,output wire [2:0]inst_type,output wire signed [31:0] alu_result,output [31:0] Read_data_memory);
wire reg_dst;
wire alu_src;
wire mem_to_reg;
wire reg_write;         			 //which is also write-enable of the reg. file
reg inst_mem_write;
wire[31:0] current_address;
reg inst_mem_read;
wire branch;
wire [1:0]alu_op;
wire jump;	
wire [31:0] D1;     		                 //first output of reg. file
wire [31:0] D2;      		                 //second output of reg. file
wire [31:0] b;       			         //output of mux between alu and reg. file
wire  [2:0] alu_ctrl_ip; 
wire zero; 					 //output of alu
reg [15:0] immediate; 				 // immediate of the instruction
reg [5:0] func;  				 //from the instruction
reg [4:0] rs; 					 //input of reg. file( rs)
reg [4:0] rt; 					 //input of reg. file (rt)
reg [4:0] rd;					 //input of Left MUX  (rd)
wire [4:0] WR_reg; 				 //input of reg. file (WR_reg)  or output of the left MUX
wire [31:0] WR_data; 				 //input of reg. file (WR_data)
wire [31:0] ext;				 //output of 'sign extend'
wire [31:0] shifted; 				 //output of shift left
//wire [31:0] Read_data_memory; 		 //output of data memory
wire [31:0] new_address;			 //input for PC 		 //output of PC
wire [31:0] pc_after_add; 			 //result of adding pc+4
wire [31:0] addr_result_case_branch;		 //the result of the adder on the top right
wire and_result_case_branch; 			 //the result of the 'and' on the top right
wire [31:0] Instruction;                         //output of instruction memory
reg [31:26] opcode;                              //opcode
wire [31:0] rt_ext;
wire [31:0] rd_ext;
reg [4:0] shamt;

/*   divide the instruction  
initial
begin

inst_mem_read=1;
inst_mem_write=0;
//#1
//func=Instruction[5:0];                  //func
//rs=Instruction[25:21];                  //rs
//rt=Instruction[20:16];                  //rt
//rd=Instruction[15:11];                  //rd
//immediate=Instruction[15:0];            //immediate
//opcode=Instruction[31:26];              //opcode
end
always@(posedge clk or Instruction)
begin 
func=Instruction[5:0];                  //func
rs=Instruction[25:21];                  //rs
rt=Instruction[20:16];                  //rt
rd=Instruction[15:11];                  //rd
immediate=Instruction[15:0];            //immediate
opcode=Instruction[31:26];              //opcode
shamt=Instruction[10:6];              //shift amount
end 
  */
//____ _____________MAKING MODULES INSTANCES IN TOP MODULE_________________

///**/Alu my_alu0(D1,b,alu_ctrl_ip,shamt,alu_result, zero);
///**/Alu_Control my_alu_control0(func,alu_op,alu_ctrl_ip);
///**/Reg_File my_reg_file0(clk,rs,rt,WR_reg,WR_data,reg_write,D1,D2);
///**/MUX my_left_mux0(rt_ext,rd_ext,reg_dst,WR_reg);   //Left mux
///**/MUX my_middle_mux0(D2,ext,alu_src,b);  // Middle mux
///**/MUX my_right_mux0(alu_result,Read_data_memory,mem_to_reg,WR_data);   //Data_memory mux
///**/MUX my_top_right_mux0(pc_after_add,addr_result_case_branch,and_result_case_branch,new_address); //mux on the top right
///**/Control my_control0(opcode,inst_type,reg_dst,alu_src,mem_to_reg,reg_write,mem_read,mem_write,branch,alu_op,jump);
///**/Data_Memory my_data_memory0(alu_result,D2,mem_write,mem_read,clk,Read_data_memory);
///**/PC my_pc0(new_address,clk,current_address);
///**/ShiftLeftBy2 my_shift_left0(ext,shifted);
///**/Adder my_top_right_adder10(shifted,pc_after_add,addr_result_case_branch); //adder at top right
///**/Adder my_top_left_adder10(current_address,1, pc_after_add);             //adder at top left
///**/And my_branch_and10(zero,branch,and_result_case_branch);
///**/Sign_Extension my_sign_extension0(immediate,ext);
///**/Reg_Sign_Ext my_sign_extension1(rt,rt_ext);
///**/Reg_Sign_Ext my_sign_extension2(rd,rd_ext);
///**/Instruction_Memory my_inst_memory0(current_address,inst_mem_write,inst_mem_read,clk,Instruction);
//
//endmodule

module TB_Top_Mips_Processor();
reg clk;
reg mem_read;
reg mem_write;
reg [31:0]current_address1;
wire [2:0]inst_type1;
wire [31:0]Read_data_mem;
wire signed [31:0] alu_res;
wire [31:0]Instruction1;

Top_Mips_Processor top_module( clk,mem_read,mem_write,inst_type1,alu_res,Read_data_mem);
always 
#50 clk <= ~clk ;      //create clk


initial
begin 
clk=0;
current_address1=0;
//mem_write=0;
//mem_read=1;


#50    
$strobe ($time,,,"%d %d %d %b",clk,alu_res,inst_type1,Read_data_mem) ;       
#100   
$strobe ($time,,,"%d %d %d %b",clk,alu_res,inst_type1,Read_data_mem) ; 


#100   
$strobe ($time,,,"%d %d %d %b",clk,alu_res,inst_type1,Read_data_mem) ; 


#100   
$strobe ($time,,,"%d %d %d %b",clk,alu_res,inst_type1,Read_data_mem) ; 


#100  
$strobe ($time,,,"%d %d %d %b",clk,alu_res,inst_type1,Read_data_mem) ; 




end


endmodule


module HazardUnit(IDRegRs,IDRegRt,EXRegRd,EXMemRead,PCWrite,IFIDWrite,HazMuxCon); 
input [4:0] IDRegRs,IDRegRt,EXRegRd; 
input EXMemRead;    
output PCWrite, IFIDWrite, HazMuxCon;         
reg PCWrite, IFIDWrite, HazMuxCon;         
always@(IDRegRs,IDRegRt,EXRegRd,EXMemRead)     
if(EXMemRead&&((EXRegRd == IDRegRs)||(EXRegRd == IDRegRt)))  //hena I check if the previous instruction was lw & if one of the two registers (rs,rt )in the current instruction   depents on rd register from the previous instruction     
begin           
PCWrite = 0;    // keep the  pc value            
IFIDWrite = 0;  // keep the IF/ID data           
HazMuxCon = 1;  // if 1 : put all control line by 0 >>> delete the instruction      
 end    
else       
begin   
// normal case // continue        
PCWrite = 1;            
IFIDWrite = 1;            
HazMuxCon = 0;             
end 
 
endmodule 

module ForwardUnit(MEMRegRd,WBRegRd,EXRegRs,EXRegRt, MEM_RegWrite, WB_RegWrite, ForwardA, ForwardB); 
   input[4:0] MEMRegRd,WBRegRd,EXRegRs,EXRegRt;  
   input MEM_RegWrite, WB_RegWrite; 
   output[1:0] ForwardA, ForwardB; 
 
   reg[1:0] ForwardA, ForwardB; 
    
   //Forward A 
   always@(MEM_RegWrite or MEMRegRd or EXRegRs or WB_RegWrite or WBRegRd) 
   begin 
      if((MEM_RegWrite)&&(MEMRegRd != 0)&&(MEMRegRd == EXRegRs)) 
         ForwardA = 2'b10; 
      else if((WB_RegWrite)&&(WBRegRd != 0)&&(WBRegRd == EXRegRs)&&(MEMRegRd != EXRegRs) ) 
         ForwardA = 2'b01; 
      else 
         ForwardA = 2'b00; 
   end 
 
   //Forward B 
   always@(WB_RegWrite or WBRegRd or EXRegRt or MEMRegRd or MEM_RegWrite) 
   begin 
      if((WB_RegWrite)&&(WBRegRd != 0)&&(WBRegRd == EXRegRt)&&(MEMRegRd != EXRegRt) ) 
         ForwardB = 2'b01; 
      else if((MEM_RegWrite)&&(MEMRegRd != 0)&&(MEMRegRd == EXRegRt)) 
         ForwardB = 2'b10; 
      else  
         ForwardB = 2'b00; 
   end 
 
endmodule



module Data_Hazard_Mux (input[11:0] control_signals,zero,input wire selector,output reg[11:0] mux_out);

always@(control_signals,zero,selector) 
begin 
  case(selector) 
      1'b0: 
        mux_out <= control_signals; 
      1'b1: 
        mux_out <= zero; 
  endcase 
end 



endmodule

module Mux_3to1(X1,X2,X3,A,Out);
input [1:0] A; 
input [31:0] X1,X2,X3; 
output reg [31:0] Out; 
 
//reg [31:0] Out; 
 
always@(A,X1,X2,X3) 
begin 

  case(A) 
      2'b00: 
        Out = X1; 
      2'b01: 
        Out = X2; 
      2'b10: 
        Out = X3; 
	default:
	Out =  X1;
  endcase 
end 
 
endmodule 







module XOR(input[31:0] Data1,input[31:0] Data2, output reg [31:0] branch);
reg [5:0] D;
always@*
begin
D=0;
while(D<32)
begin
if (Data1[D] != Data2[D])
branch[D]<=1;
else
branch[D]<=0;

D=D+1;
end
end
endmodule
module OR(input [31:0]branches, output reg branch2);
always@*
branch2 <= ~ (branches[0] | branches[1] | branches[2] | branches[3] | branches[4] | branches[5] | branches[6] | branches[7] | branches[8] | branches[9] | branches[10] | branches[11] | branches[12] | branches[13] | branches[14] | branches[15] | branches[16] | branches[17] | branches[18] | branches[19] | branches[20] | branches[21] | branches[22] | branches[23] | branches[24] | branches[25] | branches[26] | branches[27] | branches[28] | branches[29] | branches[30] | branches[31]) ;
endmodule




module IFID_pipeline_register(clk,data_hz_control,instruction_in,PCNow_in,PCNext4_in,instruction_out,PCNow_out,PCNext4_out);    

input clk,data_hz_control;     						       
input[31:0] instruction_in,PCNow_in,PCNext4_in;                //the three inputs are wire
output reg [31:0]instruction_out,PCNow_out,PCNext4_out;        //the three ouputs are reg


always@(posedge clk)						
if(data_hz_control==0)
begin 
instruction_out<=instruction_out;
PCNext4_out<=PCNext4_out;
PCNow_out<=PCNow_out;
end
else
begin 
instruction_out<=instruction_in;                           //1st output <= 1st input
PCNext4_out<=PCNext4_in;				      //2nd output <= 2nd input
PCNow_out<=PCNow_in;				       	      //3rd output <= 3rd input
end
endmodule

////////////////////////////////////////////////END OF MODULE//////////////////////////////////////////////////////////////////////

module TB_IFID_pipeline_register;

reg clk=0;
reg [31:0]instruction_in,PCNow_in,PCNext4_in;  
wire [31:0] instruction_out,PCNow_out,PCNext4_out; 

always 
#50 clk <= ~clk ;

initial
begin
#50
instruction_in=10;
PCNow_in=11;
PCNext4_in=12;
$strobe ($time,,,"%d %d %d %d %d %d %d ",clk,instruction_in,PCNow_in,PCNext4_in,instruction_out,PCNow_out,PCNext4_out);

#100
instruction_in=13;
PCNow_in=14;
PCNext4_in=15;
$strobe ($time,,,"%d %d %d %d %d %d %d ",clk,instruction_in,PCNow_in,PCNext4_in,instruction_out,PCNow_out,PCNext4_out);

end

IFID_pipeline_register R1(clk,instruction_in,PCNow_in,PCNext4_in,instruction_out,PCNow_out,PCNext4_out); 

endmodule

////////////////////////////////////////////////END OF MODULE//////////////////////////////////////////////////////////////////////

module IDEX_pipeline_register(								 

input clk,						
				 
input [11:0]in1,							 //

input [31:0]D1,							 // first output of reg_file is input to pipeline_reg

input [31:0]D2,							 // second output of reg_file is input to pipeline_reg

input [31:0]sign_extension_output,				 // output of sign_extension is input to pipeline_reg

input [4:0]shamti,rs,rt,rd,						 // rs,rt,rd 

output reg [11:0]in1_EX,						 //

output reg [31:0]D1_EX,						 // first output of reg_file is input to pipeline_reg

output reg [31:0]D2_EX,						 // second output of reg_file is input to pipeline_reg

output reg [31:0]sign_extension_output_EX,			 // output of sign_extension is input to pipeline_reg

output reg [4:0]shamt,rs_EX,rt_EX,rd_EX			         // rs,rt,rd 
);   



always@(posedge clk)						
begin
if(in1[7]==0)
begin
in1_EX<=in1;                                             	      //1st output <= 1st input
D1_EX<=D1;							      //2nd output <= 2nd input
D2_EX<=D2;				       			      //3rd output <= 3rd input
sign_extension_output_EX<=sign_extension_output;		      //4th output <= 4th input
rs_EX<=rs;						              //5th output <= 5th input
rt_EX<=rt;							      //6th output <= 6th input
rd_EX<=rd;							      //7th output <= 7th input
shamt<=shamti;
end
else
begin
in1_EX<=in1;                                             	      //1st output <= 1st input
D1_EX<=D1;							      //2nd output <= 2nd input
D2_EX<=sign_extension_output;				       			      //3rd output <= 3rd input
sign_extension_output_EX<=sign_extension_output;		      //4th output <= 4th input
rs_EX<=rs;						              //5th output <= 5th input
rt_EX<=rt;							      //6th output <= 6th input
rd_EX<=rd;	
shamt<=shamti;
end
end
endmodule

////////////////////////////////////////////////END OF MODULE//////////////////////////////////////////////////////////////////////

module TB_IDEX_pipeline_register;
reg clk=0;
reg [7:0]in1;
reg [31:0]D1;										
reg [31:0]D2;										
reg [31:0]sign_extension_output;				
reg [4:0]rs,rt,rd;

wire [7:0]in1_EX;								 
wire [31:0]D1_EX;								
wire [31:0]D2_EX;							
wire [31:0]sign_extension_output_EX;				
wire [4:0]rs_EX,rt_EX,rd_EX;					    

always 
#50 clk <= ~clk ;

initial
begin
#50
in1=2;
D1=3;
D2=4;
sign_extension_output=5;
rs=6;
rt=7;
rd=8;

$strobe ($time,,,"%d %d %d %d %d %d %d %d %d %d ",clk,in1,D1,D2,sign_extension_output,rs,rt,rd,
in1_EX,D1_EX,D2_EX,sign_extension_output_EX,rs_EX,rt_EX,rd_EX);

#100
in1=9;
D1=10;
D2=11;
sign_extension_output=12;
rs=13;
rt=14;
rd=15;
$strobe ($time,,,"%d %d %d %d %d %d %d %d %d %d ",clk,in1,D1,D2,sign_extension_output,rs,rt,rd,
in1_EX,D1_EX,D2_EX,sign_extension_output_EX,rs_EX,rt_EX,rd_EX);

end
IDEX_pipeline_register R2(clk,in1,D1,D2,sign_extension_output,rs,rt,rd,
in1_EX,D1_EX,D2_EX,sign_extension_output_EX,rs_EX,rt_EX,rd_EX);   


endmodule


////////////////////////////////////////////////END OF MODULE//////////////////////////////////////////////////////////////////////


module EXMEM_pipeline_register(								 

input clk,	
				
input [1:0]in1,						 //for control

input [1:0]in2,						 //for control


input [31:0]ALU_Result,					 //output of ALU

input [31:0]rt,						 //rt msh 3arf 32 bit leh bs alm3eed 2al kda ^^ (may be updated later)

input [4:0]rd,						 //rd (out from MUX)

output reg [1:0]in1_EX,			
		
output reg [1:0]in2_EX,			
		
output reg [31:0]ALU_Result_EX,		
			
output reg [31:0]rt_EX,		

output reg [4:0]rd_EX	        
);   


always@(posedge clk)						
begin 
in1_EX<=in1;                                          //1st output <= 1st input
in2_EX<=in2;					      //2nd output <= 2nd input
ALU_Result_EX<=ALU_Result;			      //3rd output <= 3rd input
rt_EX<=rt;					      //4th output <= 4th input
rd_EX<=rd;				              //5th output <= 7th input
end

endmodule


////////////////////////////////////////////////END OF MODULE//////////////////////////////////////////////////////////////////////

module MEMWB_pipeline_register(								 

input clk,		
								 
input [1:0]in1,					
	 //for control

input [31:0]Data_read,					 //output of Data memory

input [31:0]ALU_Result,					 //output of ALU

input [4:0]rd,						 //rd

output reg [1:0]in1_EX,					
			
output reg [31:0]Data_read_EX,		
			
output reg [31:0]ALU_Result_EX,		

output reg [4:0]rd_EX	        
);


always@(posedge clk)						
begin 

in1_EX<=in1;                                          //1st output <= 1st input
Data_read_EX<=Data_read;			      //2nd output <= 2nd input
ALU_Result_EX<=ALU_Result;			      //3rd output <= 3rd input
rd_EX<=rd;					      //4th output <= 4th input

end

endmodule

module Top_pipline( clk,inst_type,D1,D2,alu_result,Read_data_memory);
input clk;
wire reg_dst;
wire alu_src;
wire mem_to_reg;
wire reg_write;         			 //which is also write-enable of the reg. file
reg inst_mem_write;
wire[31:0] current_address;
reg  inst_mem_read;
wire branch;
wire [1:0]alu_op;
wire jump;	
output wire [31:0] D1;     		                 //first output of reg. file
output wire [31:0] D2;      		                 //second output of reg. file
wire [31:0] b;       			         //output of mux between alu and reg. file
wire  [2:0] alu_ctrl_ip;                         //input to alu ,output of alu control
wire zero; 					 //output of alu
reg [15:0] immediate; 				 // immediate of the instruction
reg [5:0] func;  				 //from the instruction
wire [4:0] shamt;
reg [4:0] rs; 					 //input of reg. file( rs)
reg [4:0] rt; 					 //input of reg. file (rt)
reg [4:0] rd;					 //input of Left MUX  (rd)
wire [4:0] WR_reg; 				 //input of reg. file (WR_reg)  or output of the left MUX
wire [31:0] WR_data; 				 //input of reg. file (WR_data)
wire [31:0] ext;				 //output of 'sign extend'
wire [31:0] shifted; 				 //output of shift left
output wire [31:0] Read_data_memory; 			 //output of data memory
wire [31:0] new_address;			 //input for PC 		 //output of PC
wire [31:0] pc_after_add; 			 //result of adding pc+4
wire [31:0] addr_result_case_branch;		 //the result of the adder on the top right
wire and_result_case_branch; 			 //the result of the 'and' on the top right
wire [31:0] Instruction;                         //output of instruction memory
reg [31:26] opcode;                              //opcode
wire [31:0] rt_ext;
wire [31:0] rd_ext;
wire [31:0]pc_input;
wire [31:0]xor_result;
wire or_comp;
wire [31:0] if_id_instruction;
wire [31:0] if_id_current_address;
wire [31:0] if_id_new_address;
wire [31:0]id_ex_D1,id_ex_D2,id_ex_ext;
wire [4:0] id_ex_rs,id_ex_rt,id_ex_rd;
wire[11:0]id_ex_control_signals;
reg [4:0]shamti;
reg [11:0] control_signals;
output wire [2:0]inst_type;
wire mem_write;
wire mem_read;
wire [31:0] alu_1st_input;
wire [31:0] alu_2nd_input;
wire [31:0] final_mux_result;
wire [1:0] ForwardA;
wire [1:0] ForwardB;
wire [31:0] ex_mem_alu_res;
wire [4:0] rt_rd_mux_out, ex_mem_rt_rd_out,mem_wb_rt_rd_out;
wire [1:0] mem_wb_write_back;
reg [1:0]ex_mem_write_back_i;
reg [1:0]ex_mem_mem_ctrls_i;
wire PCWrite;
wire IFIDWrite;
wire HazMuxCon;
wire [11:0] mux_out_control;
wire [1:0]ex_mem_write_back,ex_mem_mem_ctrls;
wire[31:0]ex_mem_alu_2nd_input;
output wire [31:0] alu_result;
//wire[4:0]ex_mem_rt_rd_out;
wire [31:0]mem_wb_read_data_memory,mem_wb_alu_res;
/*   divide the instruction  */
initial
begin

inst_mem_read=1;
inst_mem_write=0;
//#1
//func=Instruction[5:0];                  //func
//rs=Instruction[25:21];                  //rs
//rt=Instruction[20:16];                  //rt
//rd=Instruction[15:11];                  //rd
//immediate=Instruction[15:0];            //immediate
//opcode=Instruction[31:26];              //opcode
end

//assign inst_type=id_ex_control_signals[11:9];   //Edited
//assign reg_dst=id_ex_control_signals[8];       //Edited
//assign alu_src=id_ex_control_signals[7];       //bye bye
//assign mem_to_reg=id_ex_control_signals[6];   //5las
//assign reg_write=id_ex_control_signals[5];    //5las
//assign mem_read=id_ex_control_signals[4];    //5las
//assign mem_write=id_ex_control_signals[3];  //5las
//assign alu_op=id_ex_control_signals[2:1];  //5las
//assign jump=id_ex_control_signals[0];  //5las
//assign ex_mem_write_back_i={mem_to_reg,reg_write}; //5las
//assign ex_mem_mem_ctrls_i={mem_read,mem_write}; //5las
always@(posedge clk or if_id_instruction or alu_op)
begin
/**/control_signals={inst_type,reg_dst,alu_src,mem_to_reg,reg_write,mem_read,mem_write,alu_op,jump};
ex_mem_write_back_i=id_ex_control_signals[6:5];
ex_mem_mem_ctrls_i=id_ex_control_signals[4:3];
func=if_id_instruction[5:0];                  //func
rs=if_id_instruction[25:21];                  //rs
rt=if_id_instruction[20:16];                  //rt
rd=if_id_instruction[15:11];                  //rd
immediate=if_id_instruction[15:0];            //immediate
opcode=if_id_instruction[31:26];              //opcode
shamti=if_id_instruction[10:6];
end

//_________________MAKING MODULES INSTANCES IN TOP MODULE_________________

/**/Alu my_alu0(alu_1st_input,alu_2nd_input,alu_ctrl_ip,shamt,alu_result,zero); 

/**/Alu_Control my_alu_control0(id_ex_ext[5:0],id_ex_control_signals[2:1],alu_ctrl_ip);

/**/PC my_pc(pc_input,clk,PCWrite,current_address);

/**/Adder PC_counter(current_address,1,new_address); 

/**/MUX Befor_PC_mux(new_address,addr_result_case_branch,and_result_case_branch,pc_input);
   
/**/Instruction_Memory my_inst_memory(current_address,inst_mem_write,inst_mem_read,clk,Instruction);

/**/IFID_pipeline_register IFID_reg(clk,IFIDWrite,Instruction,current_address,new_address,if_id_instruction,if_id_current_address,if_id_new_address);

/**/Adder get_branch_address(if_id_new_address,shifted,addr_result_case_branch);

/**/Control my_control0(opcode,inst_type,reg_dst,alu_src,mem_to_reg,reg_write,mem_read,mem_write,branch,alu_op,jump);

/**/Reg_File my_reg_file0(clk,rs,rt,mem_wb_rt_rd_out,final_mux_result,mem_wb_write_back[0],D1,D2);	//needs to modify wr_reg and wr_data according to the mem/wr register

/**/XOR xor0(D1,D2,xor_result);

/**/OR  or0(xor_result,or_comp);

/**/And my_branch_and10(or_comp,branch,and_result_case_branch);

/**/Sign_Extension my_sign_extension0(immediate,ext);

/**/ShiftLeftBy2 my_shift_left0(ext,shifted);

/**/IDEX_pipeline_register IDEX_reg(clk,mux_out_control,D1,D2,ext,shamti,rs,rt,rd,id_ex_control_signals,id_ex_D1,id_ex_D2,
id_ex_ext,shamt,id_ex_rs,id_ex_rt,id_ex_rd);

EXMEM_pipeline_register EXMEM_reg(clk,ex_mem_write_back_i,ex_mem_mem_ctrls_i,alu_result,
alu_2nd_input,rt_rd_mux_out,ex_mem_write_back,ex_mem_mem_ctrls,ex_mem_alu_res,ex_mem_alu_2nd_input,ex_mem_rt_rd_out);

MEMWB_pipeline_register MEMWB_reg(clk,ex_mem_write_back,Read_data_memory,ex_mem_alu_res,ex_mem_rt_rd_out,
mem_wb_write_back,mem_wb_read_data_memory,mem_wb_alu_res,mem_wb_rt_rd_out);

Mux_3to1 first_alu_input(id_ex_D1,final_mux_result,ex_mem_alu_res,ForwardA,alu_1st_input);    //before alu (1)

Mux_3to1 second_alu_input(id_ex_D2,final_mux_result,ex_mem_alu_res,ForwardB,alu_2nd_input);   //before alu (2)

MUX rt_rd_mux(id_ex_rt,id_ex_rd, id_ex_control_signals[8], rt_rd_mux_out);    // al mux ally t7t al alu

MUX final_mux(mem_wb_alu_res,mem_wb_read_data_memory,mem_wb_write_back[1],final_mux_result);   

Data_Hazard_Mux hazard_mux(control_signals,12'b000000000000,HazMuxCon,mux_out_control);      //

ForwardUnit FU(ex_mem_rt_rd_out,mem_wb_rt_rd_out,id_ex_rs, id_ex_rt, ex_mem_write_back[0], mem_wb_write_back[0], ForwardA, ForwardB); 

HazardUnit hz_unit(rs,rt,id_ex_rt,id_ex_control_signals[4],PCWrite,IFIDWrite,HazMuxCon);
/**///MUX alu_input(id_ex_D2,id_ex_ext,alu_src,alu_2nd_input);
/**/Data_Memory my_data_memory(ex_mem_alu_res,ex_mem_alu_2nd_input,ex_mem_mem_ctrls[0],ex_mem_mem_ctrls[1],clk,Read_data_memory);
/**/           //adder at top left
/**/Reg_Sign_Ext my_sign_extension1(rt,rt_ext);
/**/Reg_Sign_Ext my_sign_extension2(rd,rd_ext);
endmodule

module TB_top_pipline ();
reg clk;
wire [2:0] inst_type; 
wire [31:0] alu_result;
wire [31:0] D1; 
wire [31:0] D2;
wire [1:0] forwardA;
wire [1:0] forwardB;
wire [31:0]Read_data_memory;
wire PCWrite;
wire IFIDWrite;
 integer f;
Top_pipline top_module( clk,inst_type,D1,D2,alu_result,Read_data_memory);

always 
#50 clk= ~clk;

initial
 
begin
f = $fopen("modelsim_results.txt","w");
clk=1;
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
  $fwrite(f,"%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory); 
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
 $fwrite(f,"%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory); 
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
 $fwrite(f,";%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory); 
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
 $fwrite(f,"%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory); 
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
 $fwrite(f,"%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory); 
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
 $fwrite(f,"%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory); 
#100
$strobe ($time,,,"%d %d %d %d %d",inst_type,D1,D2,alu_result,Read_data_memory) ; 
 $fwrite(f,"%d %d %d %d %d \n",inst_type,D1,D2,alu_result,Read_data_memory);
end
endmodule 
