`include "../sim/data_array/data_array_wrapper.v"
`include "../sim/tag_array/tag_array_wrapper.v"
`include "L1C_data.v"
`include "L1C_inst.v"

module top (
    input           clk,
    input           rst,

    //=================================================================
    // IO Interface for connecting to IM
    //=================================================================
    // AR channel
    output [31:0]   ARADDR_IM,
    output          ARVALID_IM,
    input           ARREADY_IM,
    // R channel
    input  [127:0]  RDATA_IM,
    input           RVALID_IM,
    output          RREADY_IM,

    //=================================================================
    // IO Interface for connecting to DM
    //=================================================================
    // AR channel
    output [31:0]   ARADDR_DM,
    output          ARVALID_DM,
    input           ARREADY_DM,
    // R channel
    input  [127:0]  RDATA_DM,
    input           RVALID_DM,
    output          RREADY_DM,

    // AW channel
    output [31:0]   AWADDR_DM,
    output          AWVALID_DM,
    input           AWREADY_DM,

    // W channel
    output [127:0]  WDATA_DM,
    output [15:0]   WSTRB_DM,
    output          WVALID_DM,
    input           WREADY_DM
);

    //-----------------------------------------------------------------------------------
    //
    // 1. 宣告 Pipeline Registers
    //
    //-----------------------------------------------------------------------------------

    // --- IF/ID Stage Registers ---
    reg [31:0] if_id_pc;
    reg [31:0] if_id_inst;
    reg core_valid_mem;  

    // --- ID/EX Stage Registers ---
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_csr_data;
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_imm_ext;
    reg [4:0]  id_ex_rd_addr;
    reg [2:0]  id_ex_funct3;
    reg [6:0]  id_ex_funct7;
    reg [4:0]  id_ex_rs1_addr; // Forwarding Unit 檢查用
    reg [4:0]  id_ex_rs2_addr;
    // 控制訊號
    reg        id_ex_reg_write;    
    reg        id_ex_mem_read;
    reg        id_ex_mem_write;
    reg        id_ex_mem_to_reg;
    reg        id_ex_csr_to_reg;
    reg        id_ex_alu_src;
    reg [3:0]  id_ex_alu_op;
    reg        id_ex_is_jal;

    // ---EX/MEM Stage Registers ---
    reg [31:0] ex_mem_csr_data;
    reg [31:0] ex_mem_alu_out;
    reg [31:0] ex_mem_store_data;
    reg [4:0]  ex_mem_rd_addr;
    reg [2:0]  ex_mem_funct3;
    reg [4:0]  ex_mem_rs1_addr; // Forwarding Unit 檢查用
    reg [4:0]  ex_mem_rs2_addr;
    // 控制訊號
    reg [3:0]  ex_mem_alu_op;
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_csr_to_reg;
    reg        ex_mem_mem_to_reg;

    // ---MEM/WB Stage Registers ---
    reg [31:0]  mem_wb_csr_data;
    reg [4:0]  mem_wb_rd_addr;
    reg [2:0]  mem_wb_funct3;
    reg [31:0] mem_wb_alu_out;
    reg [31:0] mem_wb_mem_data;
    // 控制訊號
    reg [3:0]  mem_wb_alu_op;
    reg        mem_wb_reg_write;
    reg        mem_wb_csr_to_reg;
    reg        mem_wb_mem_to_reg;


    //-----------------------------------------------------------------------------------
    //
    //  2. 宣告 Wire (連接模組用)
    //
    //-----------------------------------------------------------------------------------

    // IF Stage Wires
    reg  [31:0] pc;
    wire [31:0] pc_next;
    wire [31:0] instr_fetched;
    wire        branch_taken;
    wire [31:0] branch_target;

    // ID Stage Wires
    reg  [63:0] cycle_reg;
    reg  [63:0] instruct_reg;
    wire [31:0] CSR_data;
    wire [31:0] rs1_data_out;
    wire [31:0] rs2_data_out;
    wire [31:0] imm; 
    wire [6:0]  id_opcode = if_id_inst[6:0];    // if_id_inst 切出 opcode 給 BranchResolutionUnit 用
    wire [2:0]  id_funct3 = if_id_inst[14:12];  // if_id_inst 切出 funct3 給 BranchResolutionUnit 用

    // Decoder 輸出的控制訊號
    wire        ctrl_reg_write; 
    wire        ctrl_mem_read;
    wire        ctrl_mem_write; 
    wire        ctrl_mem_to_reg; 
    wire        ctrl_csr_to_reg;
    wire        ctrl_alu_src;
    wire [3:0]  ctrl_alu_op;
    wire        ctrl_is_jal;

    // HazardDetectionUnit 
    wire        pc_write;
    wire        if_id_write;
    wire        ctrl_flush;

    // EX Stage Wires
    wire [31:0] alu_input_a;
    wire [31:0] alu_input_b;
    wire [31:0] alu_result;
    
    // Forwarding 輸出的控制訊號
    wire [1:0]  forward_a;
    wire [1:0]  forward_b;

    // MEM Stage Wires

    // WB Stage Wires
    wire [31:0] wb_write_data;
    wire [31:0] mem_wb_mem_data_offseted;

    //-----------------------------------------------------------------------------------
    //
    //  4. 各 stage 
    //
    //-----------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------
    //  IF Stage 控制邏輯整合
    //-----------------------------------------------------------------------------------

    // I-Cache 請求：除非 Reset，否則持續拉高請求 CPU 側地址
    assign core_req = !rst;

    wire core_valid;                                                             // 定義 IF 階段的 Stall 來源： 從 IM Cache 接線出來
    wire im_stall = core_req && !core_valid;                                     // 定義 IM 忙碌 (IM Miss) - inst_cache_busy: Cache 正在處理 Miss (請求發出但 Valid 還沒回)
    wire dm_stall = (ex_mem_mem_read || ex_mem_mem_write) && !core_valid_mem;    // 定義 DM 忙碌 (DM Miss) - load_use_hazard: HazardDetectionUnit 偵測到需要 Stall (pc_write 為 0)
    wire global_stall = im_stall || dm_stall;
    // [Hazard Stall] 邏輯冒險 -> 這是 Local 的 (只影響 PC 和 IF/ID)
    // 這裡我們不需要另外定義 wire load_use_hazard，直接用 pc_write 即可

    assign pc_next = (branch_taken) ? branch_target : (pc + 32'd4);

    // PC 暫存器更新
    always @(posedge clk or posedge rst) begin
        if (rst)                
            pc <= 32'b0;
        else if (global_stall)  // 有最高優先權
            pc <= pc;       
        else if (!pc_write)     // [優先權 2] Hazard Stall (load_use_hazard)：Cache 沒事，但 HazardUnit 說 pc_write=0，也要停
            pc <= pc; 
        else if (branch_taken)  // [優先權 3] Branch Taken：跳轉
            pc <= pc_next;      
        else                    // 正常        
            pc <= pc_next;       
    end

    // 實例化 I-Cache
    L1C_inst L1C_inst_unit(
        .clk(clk),
        .rst(rst),
        .core_addr(pc),
        .core_req(core_req),
        .core_out(instr_fetched),
        .core_valid(core_valid),
        // AXI 側訊號接 top 外部 Port
        .I_ARADDR(ARADDR_IM),
        .I_ARVALID(ARVALID_IM),
        .I_ARREADY(ARREADY_IM),
        .I_RDATA(RDATA_IM),
        .I_RVALID(RVALID_IM),
        .I_RREADY(RREADY_IM)
    );

    //-----------------------------------------------------------------------------------   
    //  IF_ID Pipeline Register
    //-----------------------------------------------------------------------------------
    
    // 決定 IF/ID 暫存器是否允許寫入
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            if_id_pc   <= 32'b0;
            if_id_inst <= 32'h00000013; // 預設 NOP
        end
        else if (global_stall) begin    // 因為如果不等資料回來 (Stall) 就直接殺掉指令 (Flush)，那條還在 ID 等待數據的指令就會憑空消失，導致程式執行錯誤。
            if_id_pc   <= if_id_pc;
            if_id_inst <= if_id_inst;
        end
        else if (!if_id_write) begin
             if_id_pc   <= if_id_pc;
             if_id_inst <= if_id_inst;
        end
        // Flush (Branch 發生)
        else if (branch_taken) begin
            if_id_inst <= 32'h00000013; 
            if_id_pc   <= 32'b0;
        end
        else begin
            if_id_pc   <= pc;
            if_id_inst <= instr_fetched;
        end
    end

    //-----------------------------------------------------------------------------------
    //  ID Stage (解碼)
    //-----------------------------------------------------------------------------------

    // CSR 暫存器更新
    always@(posedge clk or posedge rst) begin
        if(rst) begin
            cycle_reg <= 63'd0;
            instruct_reg <= 63'd0;
        end
        else begin
            cycle_reg <= cycle_reg+1;
            if(pc_write)
            instruct_reg <= instruct_reg+1;
        end
    end

    // 實例化 
    CSR(
	.imm(imm),
	.cycle_reg(cycle_reg),
	.instruct_reg(instruct_reg),
	.CSR_data(CSR_data)
    );



    Decoder decoder_inst(
        .inst       (if_id_inst),
        // 輸出控制訊號
        .reg_write  (ctrl_reg_write),
        .alu_src    (ctrl_alu_src),
        .mem_read   (ctrl_mem_read),
        .mem_write  (ctrl_mem_write),
        .mem_to_reg (ctrl_mem_to_reg),
        .csr_to_reg (ctrl_csr_to_reg),
        .alu_op     (ctrl_alu_op),
        .is_jal     (ctrl_is_jal)
    );
    RegFile regfile_inst(
        .clk        (clk),
        .rst        (rst),
        .rs1_addr   (if_id_inst[19:15]),
        .rs2_addr   (if_id_inst[24:20]),
        .rd_addr    (mem_wb_rd_addr), 
        .w_data     (wb_write_data),
        .reg_write  (mem_wb_reg_write),
        .rs1_data   (rs1_data_out),
        .rs2_data   (rs2_data_out)
    );

    ImmGen immgen_inst(
        .inst(if_id_inst), // imm 的位置各不相同
        .imm(imm)          // 剛剛宣告 imm; <--- 這裡為什麼接 "imm_out"？
    );

    HazardDetectionUnit HazardDetectionUnit_inst(
        .id_ex_mem_read(id_ex_mem_read),
        .if_id_inst(if_id_inst),
        .id_ex_rd_addr(id_ex_rd_addr),
        .ex_mem_mem_read(ex_mem_mem_read),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        //.ex_mem_reg_write(ex_mem_reg_write),
        //.mem_wb_reg_write(mem_wb_reg_write),
        .pc_write(pc_write),
        .if_id_write(if_id_write),
        .ctrl_flush(ctrl_flush)
    );

    // ID Forwarding MUX rs1_data_resolved & rs2_data_resolved
    // 解析目前 ID 階段指令的 rs1, rs2 地址
    wire [4:0] id_rs1_addr = if_id_inst[19:15];
    wire [4:0] id_rs2_addr = if_id_inst[24:20];
    // 進入 BranchResolutionUnit 的資料線
    wire [31:0] id_rs1_data_resolved; 
    wire [31:0] id_rs2_data_resolved; 
    // 1. RS1 的 Forwarding MUX
    ID_Forwarding_Mux u_id_fwd_rs1 (
        .rs_addr          (id_rs1_addr),
        .reg_rdata        (rs1_data_out),     // 來自 RegFile                
        // EX Stage info
        .id_ex_rd_addr    (id_ex_rd_addr),
        .id_ex_reg_write  (id_ex_reg_write),
        .alu_result       (alu_result),       // EX 階段剛算出來的        
        // MEM Stage info
        .ex_mem_rd_addr   (ex_mem_rd_addr),
        .ex_mem_reg_write (ex_mem_reg_write),
        .ex_mem_mem_read  (ex_mem_mem_read),
        .ex_mem_alu_out   (ex_mem_alu_out),
        .mem_rdata        (RDATA_DM[31:0]),   // 注意：假設 Data Cache 回傳低 32 bits        
        // WB Stage info
        .mem_wb_rd_addr   (mem_wb_rd_addr),
        .mem_wb_reg_write (mem_wb_reg_write),
        .wb_write_data    (wb_write_data),        
        // Output
        .resolved_data    (id_rs1_data_resolved)
    );
    // 2. RS2 的 Forwarding MUX
    ID_Forwarding_Mux u_id_fwd_rs2 (
        .rs_addr          (id_rs2_addr),
        .reg_rdata        (rs2_data_out),     // 來自 RegFile       
        // EX Stage info
        .id_ex_rd_addr    (id_ex_rd_addr),
        .id_ex_reg_write  (id_ex_reg_write),
        .alu_result       (alu_result),       
        // MEM Stage info
        .ex_mem_rd_addr   (ex_mem_rd_addr),
        .ex_mem_reg_write (ex_mem_reg_write),
        .ex_mem_mem_read  (ex_mem_mem_read),
        .ex_mem_alu_out   (ex_mem_alu_out),
        .mem_rdata        (RDATA_DM[31:0]),       
        // WB Stage info
        .mem_wb_rd_addr   (mem_wb_rd_addr),
        .mem_wb_reg_write (mem_wb_reg_write),
        .wb_write_data    (wb_write_data),       
        // Output
        .resolved_data    (id_rs2_data_resolved)
    );

    BranchResolutionUnit BranchResolutionUnit_inst(
        .funct3(id_funct3),
        .opcode(id_opcode),
        .imm(imm),                       // 來自 ImmGen
        .pc(if_id_pc),                   // 來自 IF/ID PC
        .rs1_data(id_rs1_data_resolved), // [重要] 接上 Forwarding 修正後的資料
        .rs2_data(id_rs2_data_resolved), // [重要] 接上 Forwarding 修正後的資料
        .branch_taken(branch_taken),     // 輸出給 IF Stage 做 Flush
        .branch_target(branch_target)    // 輸出給 IF Stage 更新 PC
    );
    
    //-----------------------------------------------------------------------------------
    //  ID_EX Pipeline Register
    //-----------------------------------------------------------------------------------
    // --- ID/EX Pipeline Register 更新 ---
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            id_ex_pc            <= 32'b0; 
            id_ex_csr_data      <= 32'b0; 
            id_ex_rs1_data      <= 32'b0; 
            id_ex_rs2_data      <= 32'b0; 
            id_ex_imm_ext       <= 32'b0; 
            id_ex_rd_addr       <= 32'b0;
            id_ex_funct3        <=  3'b0;
            id_ex_rs1_addr      <=  5'b0;
            id_ex_rs2_addr      <=  5'b0;
            // 清空控制訊號
            id_ex_reg_write     <=  1'b0; 
            id_ex_mem_read      <=  1'b0; 
            id_ex_mem_write     <=  1'b0;
            id_ex_mem_to_reg    <=  1'b0; 
            id_ex_csr_to_reg    <=  1'b0; 
            id_ex_alu_src       <=  1'b0; 
            id_ex_alu_op        <=  1'b0;
            id_ex_is_jal        <=  1'b0;
        end 
        else if (global_stall) begin   // 最高優先權：global_stall
            // 保持原值 (Freeze)：把這裡面所有暫存器都寫一遍 <= 自己 ...
            id_ex_pc            <= id_ex_pc;
            id_ex_csr_data      <= id_ex_csr_data;
            id_ex_rs1_data      <= id_ex_rs1_data;
            id_ex_rs2_data      <= id_ex_rs2_data; 
            id_ex_imm_ext       <= id_ex_imm_ext; 
            id_ex_rd_addr       <= id_ex_rd_addr;
            id_ex_funct3        <= id_ex_funct3;
            id_ex_rs1_addr      <= id_ex_rs1_addr;
            id_ex_rs2_addr      <= id_ex_rs2_addr;
            // 控制訊號
            id_ex_reg_write     <=  id_ex_reg_write; 
            id_ex_mem_read      <=  id_ex_mem_read; 
            id_ex_mem_write     <=  id_ex_mem_write;
            id_ex_mem_to_reg    <=  id_ex_mem_to_reg; 
            id_ex_csr_to_reg    <=  id_ex_csr_to_reg; 
            id_ex_alu_src       <=  id_ex_alu_src; 
            id_ex_alu_op        <=  id_ex_alu_op;
            id_ex_is_jal        <= id_ex_is_jal;
        end
        else if (ctrl_flush) begin
            // 把所有控制訊號歸零，模擬一個 NOP
            // PC 或 Data 保持 0 或不變都沒差，重點是控制訊號要是 0
            id_ex_reg_write     <=  1'b0;  // (不寫暫存器) → 安全
            id_ex_mem_read      <=  1'b0;  // (不讀記憶體，避免 Side effect) → 安全
            id_ex_mem_write     <=  1'b0;  // (不寫記憶體) → 安全
            id_ex_mem_to_reg    <=  1'b0;  // reg_write = 0 , mem_to_reg 選擇誰不重要
            id_ex_csr_to_reg    <=  1'b0; 
            id_ex_alu_src       <=  1'b0; 
            id_ex_alu_op        <=  1'b0;  // (做加法) → 沒差，反正算出來的結果會被丟到垃圾桶。
        end
        else begin
            id_ex_pc            <= if_id_pc;
            id_ex_csr_data      <= id_ex_csr_data;
            id_ex_rs1_data      <= rs1_data_out;
            id_ex_rs2_data      <= rs2_data_out;
            id_ex_imm_ext       <= imm;
            id_ex_rd_addr       <= if_id_inst[11:7];  // rd  地址
            id_ex_funct3        <= if_id_inst[14:12];
            id_ex_rs1_addr      <= if_id_inst[19:15]; // rs1 地址
            id_ex_rs2_addr      <= if_id_inst[24:20]; // rs2 地址
            // 控制訊號
            id_ex_mem_read      <= ctrl_mem_read;   // 從 Decoder 的 output 接過來
            id_ex_mem_write     <= ctrl_mem_write;
            id_ex_mem_to_reg    <= ctrl_mem_to_reg;
            id_ex_csr_to_reg    <= ctrl_csr_to_reg;
            id_ex_alu_src       <= ctrl_alu_src;
            id_ex_reg_write     <= ctrl_reg_write;
            id_ex_alu_op        <= ctrl_alu_op;
            id_ex_is_jal        <= ctrl_is_jal;
        end
    end

    
    //-----------------------------------------------------------------------------------
    // EX stage
    //-----------------------------------------------------------------------------------

    // 實例化 ForwardingUnit
    ForwardingUnit ForwardingUnit_inst(
        .rs1_addr(id_ex_rs1_addr),       // 記得剛剛在 step 1 加的
        .rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .mem_wb_reg_write(mem_wb_reg_write),
        .forward_a(forward_a), // Forwarding or not 的控制訊號
        .forward_b(forward_b)
    );
    // ALU Input MUX
    reg [31:0]  alu_input_a_mux; // 為了在 always 裡寫加一個 reg
    reg [31:0]  rs2_resolved;    // 處理 rs2 的 Forwarding (產生一個修正後的 rs2)
    localparam  ALU_AUIPC = 4'b1011;    
    wire        is_auipc = (id_ex_alu_op == ALU_AUIPC); 
    // Input A (PC 通道): 如果是 JAL 或者是 AUIPC ，A 通道就借用 PC
    assign alu_input_a = (id_ex_is_jal | is_auipc) ? id_ex_pc : alu_input_a_mux; 
    // Input B: 只有 JAL 強制用 4 (AUIPC 會因為 is_jal=0 而走後面的邏輯，選到 imm)
    assign alu_input_b = (id_ex_is_jal) ? 32'd4 : ((id_ex_alu_src) ? id_ex_imm_ext : rs2_resolved);
    always @(*) begin
        case (forward_a)
            2'b00:      alu_input_a_mux = id_ex_rs1_data; // alu_input_a = id_ex_rs1_data;
            2'b01:      alu_input_a_mux = ex_mem_alu_out;
            2'b10:      alu_input_a_mux = wb_write_data;  // 接這條線才萬無一失 mem_wb_alu_out;
            default:    alu_input_a_mux = id_ex_rs1_data;
        endcase
        case (forward_b)
            2'b00:      rs2_resolved = id_ex_rs2_data; // assign alu_input_b = (id_ex_alu_src) ? id_ex_imm_ext : id_ex_rs2_data;
            2'b01:      rs2_resolved = ex_mem_alu_out;
            2'b10:      rs2_resolved = wb_write_data;  // 接這條線才萬無一失 // mem_wb_alu_out;
            default:    rs2_resolved = id_ex_rs2_data;
        endcase
    end
    // 實例化 ALU
    ALU ALU_inst(
        .alu_input_a(alu_input_a),
        .alu_input_b(alu_input_b),
        .alu_op(id_ex_alu_op),
        .alu_result(alu_result),
        .zero()
    );


    
    //-----------------------------------------------------------------------------------
    //  EX_MEM Pipeline Register
    //-----------------------------------------------------------------------------------

    // --- EX/MEM Pipeline Register 更新 ---
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            ex_mem_csr_data     <= 32'b0;
            ex_mem_alu_out      <= 32'b0;
            ex_mem_store_data   <= 32'b0;
            ex_mem_rd_addr      <= 32'b0;
            ex_mem_funct3       <=  3'b0;
            ex_mem_rs1_addr     <=  5'b0;
            ex_mem_rs2_addr     <=  5'b0;
            // 清空控制訊號
            ex_mem_alu_op       <=  4'b0;
            ex_mem_reg_write    <=  1'b0;
            ex_mem_mem_read     <=  1'b0;
            ex_mem_mem_write    <=  1'b0;
            ex_mem_mem_to_reg   <=  1'b0;
            ex_mem_csr_to_reg   <=  1'b0;
        end 
        else if (global_stall) begin
            ex_mem_csr_data     <= ex_mem_csr_data;
            ex_mem_alu_out      <= ex_mem_alu_out;
            ex_mem_store_data   <= ex_mem_store_data;
            ex_mem_rd_addr      <= ex_mem_rd_addr;
            ex_mem_funct3       <= ex_mem_funct3;
            ex_mem_rs1_addr     <= ex_mem_rs1_addr;
            ex_mem_rs2_addr     <= ex_mem_rs2_addr;
            // 控制訊號
            ex_mem_alu_op       <= ex_mem_alu_op;
            ex_mem_reg_write    <= ex_mem_reg_write;
            ex_mem_mem_read     <= ex_mem_mem_read;
            ex_mem_mem_write    <= ex_mem_mem_write;
            ex_mem_mem_to_reg   <= ex_mem_mem_to_reg;
            ex_mem_csr_to_reg   <= ex_mem_csr_to_reg;
        end
        else begin
            ex_mem_csr_data     <= ex_mem_csr_data;
            ex_mem_alu_out      <= alu_result;
            ex_mem_store_data   <= rs2_resolved;   // store data 是 rs2 值 bypass ， 也要考慮 forwarding !
            ex_mem_rd_addr      <= id_ex_rd_addr;  // rd  地址
            ex_mem_funct3       <= id_ex_funct3;
            ex_mem_rs1_addr     <= id_ex_rs1_addr; // rs1 地址
            ex_mem_rs2_addr     <= id_ex_rs2_addr; // rs2 地址
            // 控制訊號
            ex_mem_alu_op       <=  id_ex_alu_op;
            ex_mem_reg_write    <=  id_ex_reg_write;
            ex_mem_mem_read     <=  id_ex_mem_read;
            ex_mem_mem_write    <=  id_ex_mem_write;
            ex_mem_mem_to_reg   <=  id_ex_mem_to_reg;
            ex_mem_csr_to_reg   <=  id_ex_csr_to_reg;
        end 
    end

    
    //-----------------------------------------------------------------------------------
    // MEM stage
    //-----------------------------------------------------------------------------------

    // --- 連接 DM 介面 (這裡大概率會是錯的 debug 要很小心這裡!!) ---
    // 讀取邏輯 (Load)
    /*
    assign ARADDR_DM  = ex_mem_alu_out;     // 地址來自 ALU
    assign ARVALID_DM = ex_mem_mem_read;    // 只有 Load 指令才讀
    assign RREADY_DM  = 1'b1;               // 永遠準備好
    // 寫入邏輯 (Store)
    assign AWADDR_DM  = ex_mem_alu_out;     // 寫地址
    assign AWVALID_DM = ex_mem_mem_write;   // 只有 Store 指令才寫
    assign WDATA_DM   = {96'b0, ex_mem_store_data}; // 寫資料 (補滿128bit)
    assign WVALID_DM  = ex_mem_mem_write;
    assign WREADY_DM  = 1'b1; // 假設
    assign WSTRB_DM   = (ex_mem_mem_write) ? 16'hFFFF : 16'h0; // 簡單全開，之後要依據 SW/SH/SB 改
    */

    // 遮罩邏輯
    wire [1:0]  offset = ex_mem_alu_out[1:0];
    reg  [31:0] be;
    always @(*) begin
        case (ex_mem_funct3)
            3'b000: begin
                // -------------------------
                // SB (store byte)
                // -------------------------
                case (offset)
                    2'b00: be = 32'h000000FF;
                    2'b01: be = 32'h0000FF00;
                    2'b10: be = 32'h00FF0000;
                    2'b11: be = 32'hFF000000;
                endcase	
            end

            3'b001: begin
                // -------------------------
                // SH (store halfword)
                // -------------------------
                case (offset)
                    2'b00: be = 32'h0000FFFF;      // 對齊
                    2'b10: be = 32'hFFFF0000;      // 對齊
                    default: be = 32'h00000000;    // misaligned，視老師規定處理
                endcase
            end

            3'b010: begin
                // -------------------------
                // SW (store word)
                // -------------------------
                if (offset == 2'b00)
                    be = 32'hFFFFFFFF;             // 完整 4 bytes
                else
                    be = 32'h00000000;             // misaligned
            end
            default: be = 32'h00000000;
        endcase
    end


    wire [31:0] mem_data;
    L1C_data L1C_data_unit (
        .clk        (clk),
        .rst        (rst),

        // Core Interface
        .core_addr  (ex_mem_alu_out),
        .core_req   (ex_mem_mem_read || ex_mem_mem_write),
        .core_in    (ex_mem_store_data << (8 * offset)),
        .core_out   (mem_data),
        .core_wen   (ex_mem_mem_write),
        .core_wstrb (~be),
        .core_valid (core_valid_mem),

        // AXI Interface
        .ARADDR_DM  (ARADDR_DM),
        .ARVALID_DM (ARVALID_DM),
        .ARREADY_DM (ARREADY_DM),
        .RDATA_DM   (RDATA_DM),
        .RVALID_DM  (RVALID_DM),
        .RREADY_DM  (RREADY_DM),

        .AWADDR_DM  (AWADDR_DM),
        .AWVALID_DM (AWVALID_DM),
        .AWREADY_DM (AWREADY_DM),
        .WDATA_DM   (WDATA_DM),
        .WSTRB_DM   (WSTRB_DM),
        .WVALID_DM  (WVALID_DM),
        .WREADY_DM  (WREADY_DM)
    );
    
    //-----------------------------------------------------------------------------------
    //  MEM_WB Pipeline Register 
    //-----------------------------------------------------------------------------------
    // --- MEM/WB Pipeline Register 更新 ---
    

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mem_wb_csr_data     <= 32'b0; 
            mem_wb_alu_out      <= 32'b0; 
            mem_wb_mem_data     <= 32'b0; 
            mem_wb_rd_addr      <=  5'b0;
            mem_wb_funct3       <=  3'b0;
            // 清空控制訊號
            mem_wb_alu_op       <=  4'b0;
            mem_wb_reg_write    <=  1'b0; 
            mem_wb_mem_to_reg   <=  1'b0;
            mem_wb_csr_to_reg   <=  1'b0;
        end 
        else if (global_stall) begin
            mem_wb_csr_data     <= mem_wb_csr_data; 
            mem_wb_alu_out      <= mem_wb_alu_out; 
            mem_wb_mem_data     <= mem_wb_mem_data; // 假設資料在低 32-bit 
            mem_wb_funct3       <= mem_wb_funct3;
            // 控制訊號
            mem_wb_alu_op       <= mem_wb_alu_op;
            mem_wb_reg_write    <= mem_wb_reg_write; 
            mem_wb_mem_to_reg   <= mem_wb_mem_to_reg;
            mem_wb_csr_to_reg   <= mem_wb_csr_to_reg;
        end
        else begin
            mem_wb_csr_data     <= mem_wb_csr_data; 
            mem_wb_alu_out      <= ex_mem_alu_out; 
            mem_wb_mem_data     <= mem_data; // 假設資料在低 32-bit 
            mem_wb_rd_addr      <= ex_mem_rd_addr;
            mem_wb_funct3       <= ex_mem_funct3;
            // 控制訊號
            mem_wb_alu_op       <= ex_mem_alu_op;
            mem_wb_reg_write    <= ex_mem_reg_write; 
            mem_wb_mem_to_reg   <= ex_mem_mem_to_reg;
            mem_wb_csr_to_reg   <= ex_mem_csr_to_reg;
        end
    end
   
    //-----------------------------------------------------------------------------------
    // Write Back MUX 
    //-----------------------------------------------------------------------------------
    LD_filter LD_filter_inst(
        .funct3(mem_wb_funct3),
        .DM_data(mem_wb_mem_data),
        .LD_filter_data(mem_wb_mem_data_offseted)
    );
    assign wb_write_data = (mem_wb_mem_to_reg) ? mem_wb_mem_data_offseted : (mem_wb_csr_to_reg) ? mem_wb_csr_data : mem_wb_alu_out;
    // 在 WB Stage 執行 存回暫存器，已經接線正確地實例化 RegFile 了，不要在 top 裡畫蛇添足
endmodule

module Decoder(
    input      [31:0] inst,
    output reg        reg_write,
    output reg        alu_src,
    output reg        mem_read,
    output reg        mem_write,
    output reg        mem_to_reg,
    output reg        csr_to_reg,
    // output reg        imm_type,
    output reg [3:0]  alu_op,
    output reg        is_jal  
);
    wire [6:0] opcode = inst[6:0];      
    wire [2:0] funct3 = inst[14:12];    
    wire [6:0] funct7 = inst[31:25];    
    // ALU Operations (必須與 ALU module 裡的定義完全一樣！)
    localparam ALU_ADD      = 4'b0000;
    localparam ALU_SUB      = 4'b0001;
    localparam ALU_SLL      = 4'b0010;
    localparam ALU_SLT      = 4'b0011;
    localparam ALU_SLTU     = 4'b0100;
    localparam ALU_XOR      = 4'b0101;
    localparam ALU_SRL      = 4'b0110;
    localparam ALU_SRA      = 4'b0111;
    localparam ALU_OR       = 4'b1000;
    localparam ALU_AND      = 4'b1001;
    localparam ALU_LUI      = 4'b1010;
    localparam ALU_AUIPC    = 4'b1011;
    localparam ALU_MUL      = 4'b1100;
    localparam ALU_MULH     = 4'b1101;
    localparam ALU_MULHU    = 4'b1110;
    localparam ALU_MULHSU   = 4'b1111;
    
    // Opcode Definitions
    localparam OP_R_TYPE    = 7'b0110011;
    localparam OP_I_TYPE    = 7'b0010011;
    localparam OP_LOAD      = 7'b0000011;
    localparam OP_STORE     = 7'b0100011;
    localparam OP_BRANCH    = 7'b1100011;
    // JAL/JALR
    localparam OP_JAL       = 7'b1101111;
    localparam OP_JALR      = 7'b1100111;
    localparam OP_LUI       = 7'b0110111;
    // AUIPC
    localparam OP_AUIPC     = 7'b0010111;
    // CSR
    localparam OP_CSR       = 7'b1110011;

    always @(*) begin
        reg_write   = 1'b0;
        alu_src     = 1'b0;
        mem_read    = 1'b0;
        mem_write   = 1'b0;
        mem_to_reg  = 1'b0;
        csr_to_reg = 1'b0;
        alu_op      = ALU_ADD;
        is_jal      = 1'b0; 
        case (opcode) 
            OP_R_TYPE: begin 
                reg_write = 1'b1;
                alu_src   = 1'b0; // rs2
		if(funct7 == 7'b0000001) begin
		        case (funct3)
		            3'b000: alu_op = ALU_MUL;
		            3'b001: alu_op = ALU_MULH;
		            3'b010: alu_op = ALU_MULHU;
		            3'b011: alu_op = ALU_MULHSU;
		        endcase
		end
		else begin
		        case (funct3)
		            3'b000: alu_op = (funct7[5]) ? ALU_SUB : ALU_ADD; // 0:ADD, 1:SUB
		            3'b001: alu_op = ALU_SLL;
		            3'b010: alu_op = ALU_SLT;
		            3'b011: alu_op = ALU_SLTU;
		            3'b100: alu_op = ALU_XOR;
		            3'b101: alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL; // 0:SRL, 1:SRA
		            3'b110: alu_op = ALU_OR;
		            3'b111: alu_op = ALU_AND;
		        endcase
		end
            end
            OP_I_TYPE: begin 
                reg_write = 1'b1;
                alu_src   = 1'b1; // Imm
                case (funct3)
                    3'b000: alu_op = ALU_ADD; // ADDI
                    3'b001: alu_op = ALU_SLL; // SLLI
                    3'b010: alu_op = ALU_SLT; // SLTI
                    3'b011: alu_op = ALU_SLTU;// SLTIU
                    3'b100: alu_op = ALU_XOR; // XORI
                    3'b101: alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL; // SRLI/SRAI
                    3'b110: alu_op = ALU_OR;  // ORI
                    3'b111: alu_op = ALU_AND; // ANDI
                endcase
            end
            OP_LOAD: begin 
                reg_write  = 1'b1;
                mem_read   = 1'b1;
                mem_to_reg = 1'b1; 
                alu_src    = 1'b1; 
                alu_op     = ALU_ADD; 
            end
            OP_STORE: begin 
                mem_write = 1'b1;
                alu_src   = 1'b1; 
                alu_op    = ALU_ADD; 
            end
            OP_LUI: begin
                reg_write = 1'b1;
                alu_src   = 1'b1; // Imm (LUI 的立即數)
                alu_op    = ALU_LUI; // 特殊 OP：直接把 B (Imm) 傳出去，忽略 A
            end
            OP_AUIPC: begin
                reg_write = 1'b1;
                alu_src   = 1'b1;
                alu_op    = ALU_AUIPC; 
            end
            OP_JAL: begin
                reg_write = 1'b1;
                is_jal    = 1'b1; 
                alu_op    = ALU_ADD; 
            end
            OP_JALR: begin
                reg_write = 1'b1;
                is_jal    = 1'b1; 
                alu_src   = 1'b1; // JALR 是 rs1 + Imm
                alu_op    = ALU_ADD; 
            end    
            // Branch 指令雖然不寫回，但需要 ALU 比較 (Phase 2 Branch Prediction 會用到)
            OP_BRANCH: begin
                alu_src = 1'b0; // rs1 vs rs2
                alu_op  = ALU_SUB; // 用減法做比較
            end
            OP_CSR: begin
                csr_to_reg = 1'b1;
            end
            default: begin 
            end
        endcase
    end
endmodule

module RegFile(
    input          clk,      
    input          rst,        
    input   [4:0]  rs1_addr,  // 不是 32-bits
    input   [4:0]  rs2_addr,   
    input   [4:0]  rd_addr,    
    input   [31:0] w_data,     
    input          reg_write,  
    output  [31:0] rs1_data,   
    output  [31:0] rs2_data
);
    reg [31:0] reg_file [0:31];
    integer i; // 用來跑迴圈 reset 用的
    // 寫入邏輯 (Sequential - 只信任 clk edge)
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            reg_file[0] <= 0;
        end 
        else begin 
            // 目標地址 rd_addr 不可以是 0 (x0 永遠是 0，不能被改寫!)
            if (reg_write && rd_addr != 5'b0) begin
                reg_file[rd_addr] <= w_data;
            end
        end
    end
    assign rs1_data = (reg_write && (rd_addr != 5'b0) && (rd_addr == rs1_addr)) ? w_data : reg_file[rs1_addr];
    assign rs2_data = (reg_write && (rd_addr != 5'b0) && (rd_addr == rs2_addr)) ? w_data : reg_file[rs2_addr];
endmodule

module ImmGen(
    input      [31:0] inst,
    output reg [31:0] imm
);
    // 切出 Opcode 來判斷是什麼 Type
    wire [6:0] opcode = inst[6:0];

    // I-Type: inst[31:20]
    wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
  
    // S-Type: inst[31:25] + inst[11:7]
    wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};

    // B-Type (Branch): 比較複雜，位置很亂，且最後一位補 0
    // 順序: bit 12, bit 10:5, bit 4:1, bit 11, 0
    wire [31:0] imm_b = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};
    
    // U-Type (LUI, AUIPC): 放在高位 [31:12]，低位補 0
    wire [31:0] imm_u = {inst[31:12], 12'b0};

    // J-Type (JAL): 也是很亂
    // 順序: bit 20, bit 10:1, bit 11, bit 19:12, 0
    wire [31:0] imm_j = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};

    always @(*) begin
        case (opcode) 
            // I-Type 指令 (ADDI, LW, JALR 等)
            7'b0010011, // OP-IMM (ADDI...)
            7'b0000011, // LOAD (LW...)
            7'b1100111: // JALR
                imm = imm_i;
            // S-Type 指令 (SW 等)
            7'b0100011: // STORE
                imm = imm_s;
            // B-Type 指令 (BEQ, BNE 等) - Phase 2 會用到
            7'b1100011: // BRANCH
                imm = imm_b;           
            // U-Type 指令 (LUI, AUIPC)
            7'b0110111, // LUI
            7'b0010111: // AUIPC
                imm = imm_u;
            // J-Type 指令 (JAL)
            7'b1101111: // JAL
                imm = imm_j;
            default: 
                imm = 32'b0; // R-Type 不需要立即數，輸出 0 即可
        endcase 
    end
endmodule

module ALU(
    input      [31:0] alu_input_a,
    input      [31:0] alu_input_b,
    input      [3:0]  alu_op,
    output reg [31:0] alu_result,
    output            zero  // 順便做出來，以後 Branch 會感謝你
);
    // alu_op : 定義 ALU 操作碼 (必須與 Decoder 一致)
    localparam ALU_ADD      = 4'b0000;
    localparam ALU_SUB      = 4'b0001;
    localparam ALU_SLL      = 4'b0010;
    localparam ALU_SLT      = 4'b0011;
    localparam ALU_SLTU     = 4'b0100;
    localparam ALU_XOR      = 4'b0101;
    localparam ALU_SRL      = 4'b0110;
    localparam ALU_SRA      = 4'b0111;
    localparam ALU_OR       = 4'b1000;
    localparam ALU_AND      = 4'b1001;
    localparam ALU_LUI      = 4'b1010; // 專門給 LUI 用：直接輸出 Input B
    localparam ALU_AUIPC    = 4'b1011;
    localparam ALU_MUL      = 4'b1100;
    localparam ALU_MULH     = 4'b1101;
    localparam ALU_MULHU    = 4'b1110;
    localparam ALU_MULHSU   = 4'b1111;

    reg  [63:0]multiplier;

    assign zero = (alu_result == 32'b0);

    always @(*) begin
        alu_result = 32'b0; 
        case (alu_op)
            ALU_ADD:    alu_result = alu_input_a + alu_input_b;
            ALU_SUB:    alu_result = alu_input_a - alu_input_b;
            ALU_AND:    alu_result = alu_input_a & alu_input_b;
            ALU_OR:     alu_result = alu_input_a | alu_input_b;
            ALU_XOR:    alu_result = alu_input_a ^ alu_input_b;
            
            // 移位運算 (只取 shift amount 的低 5 位)
            ALU_SLL:    alu_result = alu_input_a << alu_input_b[4:0];
            ALU_SRL:    alu_result = alu_input_a >> alu_input_b[4:0];
            ALU_SRA:    alu_result = $signed(alu_input_a) >>> alu_input_b[4:0]; // 算術右移 (補 1)

            // 比較運算 (Set Less Than)
            // SLT: 有號數比較
            ALU_SLT:    alu_result = ($signed(alu_input_a) < $signed(alu_input_b)) ? 32'd1 : 32'd0;
            // SLTU: 無號數比較
            ALU_SLTU:   alu_result = (alu_input_a < alu_input_b) ? 32'd1 : 32'd0;

            // LUI 專用 (Pass Input B)
            ALU_LUI:    alu_result = alu_input_b; 

            // 為了 is_alupc 增加多個 reg 不是最優化的方法，有更好的方法:定義專屬的 ALU Opcode，讓 EX 階段看到這個 Opcode 時，自動知道「Input A 要選 PC」。
            ALU_AUIPC:  alu_result = alu_input_a + alu_input_b; // 功能跟 ADD 一樣
            // --- 乘法運算實作 ---
            ALU_MUL: begin
                alu_result = alu_input_a * alu_input_b;
            end
            ALU_MULH: begin
            // $signed 會自動進行符號擴展至 64-bit 乘法
                alu_result = ($signed(alu_input_a) * $signed(alu_input_b)) >>> 32;
            end
            ALU_MULHU: begin
            // 無符號乘法取高位
                multiplier = $signed(alu_input_a) * $signed({1'b0,alu_input_b});
		alu_result = multiplier[63:32];
            end
            ALU_MULHSU: begin
            // 有符號(a) * 無符號(b)
		multiplier = alu_input_a * alu_input_b;
		alu_result = multiplier[63:32];
            end
            default:  alu_result = 32'b0;
        endcase
    end
endmodule

module ForwardingUnit(
    input [4:0] rs1_addr,           // ex 階段 需 forward
    input [4:0] rs2_addr,   
    input [4:0] ex_mem_rd_addr,     // 上個指令
    input       ex_mem_reg_write,
    input [4:0] mem_wb_rd_addr,     // 上上個指令
    input       mem_wb_reg_write,
    output reg [1:0] forward_a,     // 控制 rs1
    output reg [1:0] forward_b      // 控制 rs2
);
    localparam no_hazard  = 2'b00; 
    localparam mem_hazard = 2'b01;  // MEM Forward (最優先) data_hazard_occur_at_mem_stage
    localparam wb_hazard  = 2'b10;  // WB  Forward (次優先) data_hazard_occur_at_wb_stage
    
    always @(*) begin
        forward_a = no_hazard; // 預設：不轉發
        if(ex_mem_reg_write && (ex_mem_rd_addr!=0) && (ex_mem_rd_addr==rs1_addr))
            forward_a = mem_hazard;
        else if(mem_wb_reg_write && (mem_wb_rd_addr!=0) && (mem_wb_rd_addr==rs1_addr))
            forward_a = wb_hazard;
        
        forward_b = no_hazard; // 預設：不轉發
        if(ex_mem_reg_write && (ex_mem_rd_addr!=0) && (ex_mem_rd_addr==rs2_addr))
            forward_b = mem_hazard;
        else if(mem_wb_reg_write && (mem_wb_rd_addr!=0) && (mem_wb_rd_addr==rs2_addr))
            forward_b = wb_hazard;
    end
endmodule 

module HazardDetectionUnit(
    input        id_ex_mem_read,
    input [31:0] if_id_inst,
    input [4:0]  id_ex_rd_addr,
    // 檢查前面指令有沒有 LW data hazard
    input        ex_mem_mem_read,
    input [4:0]  ex_mem_rd_addr,
    // 輸出控制訊號
    // 動作 1: Stall (踩煞車) -> 叫前面的舞台不要動
    output reg pc_write,
    output reg if_id_write,
    // 動作 2: Flush (清空) -> 往後面的舞台送氣泡 (NOP)
    output reg ctrl_flush
); 
    wire [6:0] opcode       = if_id_inst[6:0];
    wire [4:0] rs1_addr     = if_id_inst[19:15];
    wire [4:0] rs2_addr     = if_id_inst[24:20];
    // Branch 指令 (B-Type) 或是 JALR (1100111) ，要特別小心處理資料相依
    wire       b_type       = (opcode == 7'b1100011); 
    wire       jalr         = (opcode == 7'b1100111);
    wire       is_branch    = (b_type || jalr);
    always @(*) begin
        // 預設值：不 Stall
        pc_write    = 1'b1;
        if_id_write = 1'b1;
        ctrl_flush  = 1'b0;

        // ==========================================================
        // 條件 A: 標準 Load-Use Hazard (Distance 1)
        // ==========================================================
        // 上一條指令 (EX) 是 Load，且寫入目標等於目前的 rs1 或 rs2。
        // 這會擋住所有需要用 rs1/rs2 的指令 (包含 ADD, SUB, 以及 Branch)
        if (id_ex_mem_read && (id_ex_rd_addr == rs1_addr || id_ex_rd_addr == rs2_addr)) begin
            pc_write    = 1'b0;
            if_id_write = 1'b0;
            ctrl_flush  = 1'b1;
        end
        
        // ==========================================================
        // 條件 B: Branch Specific Load-Use Hazard (Distance 2)
        // ==========================================================
        // 上上條指令 (MEM) 是 Load，且目前的 ID 是 Branch。
        // 不從 MEM 拉線回 ID (怕 SRAM mem access 太慢)，多 Stall 一個 Cycle，等 Load 走到 WB 階段。
        else if (ex_mem_mem_read && (ex_mem_rd_addr == rs1_addr || ex_mem_rd_addr == rs2_addr) && is_branch) begin
            // [關鍵] 這裡用 is_branch，而不是 branch_taken！
            pc_write    = 1'b0;
            if_id_write = 1'b0;
            ctrl_flush  = 1'b1;
        end
    end
endmodule

module ID_Forwarding_Mux(           // 進入 BranchResolutionUnit 的 rs1_data & rs2_data 都可以使用的 "選擇器(MUX)"
    input  [4:0]  rs_addr,          // 來源暫存器地址 (rs1 or rs2)
    input  [31:0] reg_rdata,        // RegFile 讀出來的原始資料
    
    // 來自 EX 階段的資訊 (Priority 1 - 最鮮活的資料)
    input  [4:0]  id_ex_rd_addr,
    input         id_ex_reg_write,
    input  [31:0] alu_result,       // 來自 ALU 的計算結果
    
    // 來自 MEM 階段的資訊 (Priority 2)
    input  [4:0]  ex_mem_rd_addr,
    input         ex_mem_reg_write,
    input         ex_mem_mem_read,  // 判斷是否為 Load 指令
    input  [31:0] ex_mem_alu_out,   // ALU 運算結果傳遞
    input  [31:0] mem_rdata,        // Memory 讀出的資料 (從 DM 來的)
    
    // 來自 WB 階段的資訊 (Priority 3)
    input  [4:0]  mem_wb_rd_addr,
    input         mem_wb_reg_write,
    input  [31:0] wb_write_data,    // 準備寫回 RegFile 的資料

    output reg [31:0] resolved_data // 最終決定的資料 (給 Branch Unit 用)
);

    always @(*) begin
        // Default: 沒人用這顆暫存器，直接用 RegFile 讀到的值
        resolved_data = reg_rdata;

        // Priority 1: Forward from EX Stage (Aggressive Forwarding)
        // 情況：上一條指令剛算完 (ALU)，還沒寫回
        if (id_ex_reg_write && (id_ex_rd_addr != 0) && (id_ex_rd_addr == rs_addr)) begin
            resolved_data = alu_result;
        end
        
        // Priority 2: Forward from MEM Stage
        // 情況：上上條指令在 MEM 階段
        else if (ex_mem_reg_write && (ex_mem_rd_addr != 0) && (ex_mem_rd_addr == rs_addr)) begin
            if (ex_mem_mem_read) 
                resolved_data = mem_rdata;      // 如果是 Load，取 Memory 資料
            else                 
                resolved_data = ex_mem_alu_out; // 如果是 ALU 運算，取 ALU 結果
        end
        
        // Priority 3: Forward from WB Stage
        // 情況：上上上條指令在 WB 階段 (正在寫回)
        else if (mem_wb_reg_write && (mem_wb_rd_addr != 0) && (mem_wb_rd_addr == rs_addr)) begin
            resolved_data = wb_write_data;
        end
    end

endmodule

module BranchResolutionUnit ( 
    input [2:0]   funct3,   
    input [6:0]   opcode,        
    input [31:0]  imm,              // 來自 ImmGen
    input [31:0]  pc,               // 來自 IF/ID PC (為了算 JAL/Branch target)
    input [31:0]  rs1_data,         // 來自 forwarding MUX 選擇過 ???
    input [31:0]  rs2_data,
    output reg    branch_taken,     // 告訴 Top Level 要不要 Flush
    output [31:0] branch_target     // 告訴 PC Mux 下一跳去哪
);
    // 優化：branch_target共用同一個加法器
    wire    [31:0]  base_addr       = (opcode == 7'b1100111) ? rs1_data : pc;
    assign          branch_target   = (base_addr + imm) & ~32'd1; // 強制將最低位設為 0

    // [優化] Shared Subtractor & Flag Generation
    // 我們多加 1-bit 來處理 Unsigned 的借位 (Carry/Borrow) 
    wire    [32:0]  sub_res         = {1'b0, rs1_data} - {1'b0, rs2_data};
    // 提取 Flags
    wire            z_flag          = (sub_res[31:0] == 0);      // Zero (相等)
    wire            n_flag          = sub_res[31];               // Negative (結果為負)
    wire            v_flag          = (rs1_data[31] != rs2_data[31]) && (sub_res[31] != rs1_data[31]); // Overflow (有號數溢位)
    wire            c_flag          = sub_res[32];               // Carry/Borrow (無號數借位: 1代表沒借位(A>=B), 0代表有借位(A<B))
    // 註：Verilog 的減法借位定義可能因工具而異，
    // 更穩的 Unsigned 比較是直接看 sub_res[32] (如果 A-B 產生借位，最高位會變)
    // 或是直接用 Verilog 的 < 但限制在一個 expression 裡。
    // 這裡為了絕對的面積優化，我們用邏輯推導：
    
    // Signed Less Than (SLT): (N != V)
    wire            lt_signed       = (n_flag != v_flag);
    // Unsigned Less Than (SLTU): A < B 等同於 A - B 產生借位 (Sub result is negative in extended bit if treated right)
    // 簡單寫法： rs1 < rs2
    wire            lt_unsigned     = (rs1_data < rs2_data); // 讓工具去合這個，通常很小

    always @(*) begin
        branch_taken = 1'b0;  // 預設：沒 mispredict（predict not taken）
        case (opcode)
            7'b1101111: branch_taken = 1'b1; // JAL
            7'b1100111: branch_taken = 1'b1; // JALR: rd=PC+4, PC=imm+rs1
            7'b1100011: begin
                case (funct3)
                    3'b000: branch_taken = z_flag;          // (rs1_data == rs2_data);                      // BEQ
                    3'b001: branch_taken = !z_flag;         // (rs1_data != rs2_data);                      // BNE
                    3'b100: branch_taken = lt_signed;       // ($signed(rs1_data) <  $signed(rs2_data));    // BLT
                    3'b101: branch_taken = !lt_signed;      // ($signed(rs1_data) >= $signed(rs2_data));    // BGE
                    3'b110: branch_taken = lt_unsigned;     // (rs1_data <  rs2_data);                      // BLTU (Unsigned)
                    3'b111: branch_taken = !lt_unsigned;    // (rs1_data >= rs2_data);                      // BGEU (Unsigned)
                    default: branch_taken = 1'b0;
                endcase
            end
            default: branch_taken = 1'b0;
        endcase
end
endmodule


module LD_filter(
	input  [2:0]funct3,
	input  [31:0]DM_data,
	output reg[31:0]LD_filter_data
);

always@ (*) begin
	case(funct3) 
		3'b010:     LD_filter_data = DM_data;                               // LW
		3'b000:     LD_filter_data = {{24{DM_data[7]}}, DM_data[7:0]};      // LH
		3'b001:     LD_filter_data = {{16{DM_data[15]}}, DM_data[15:0]};    // LB
		3'b100:     LD_filter_data = {24'd0, DM_data[7:0]};                 // LBU
		3'b101:     LD_filter_data = {16'd0, DM_data[15:0]};                // LHU    
		default:    LD_filter_data = DM_data;
	endcase
end

endmodule

module CSR(
	input  [11:0] imm,
	input  [63:0] cycle_reg,
	input  [63:0] instruct_reg,
	output reg[31:0] CSR_data
);

always@ (*) begin
	case(imm)
		12'b110010000010:begin
			CSR_data = instruct_reg[63:32];
		end
		12'b110000000010:begin
			CSR_data = instruct_reg[31:0];
		end
		12'b110010000000:begin
			CSR_data = cycle_reg[63:32];
		end
		12'b110000000000:begin
			CSR_data = cycle_reg[31:0];
		end
	endcase
end

endmodule


