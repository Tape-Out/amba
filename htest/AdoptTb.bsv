package AdoptTb;

import Vector::*;
import RegIf::*;
import Apb4::*;

// 一个照 IHI 0024D 走的假从设备，输出全部过寄存器。
//
// 为什么不用现成的 mkApb4Bind 当对手：那一个的 PRDATA 是从 PADDR 组合出来的，
// 而收编器要在自己的规则里一边驱动引脚一边读回 PRDATA——同一条线又写又读，
// G0004。真的第三方是 BVI 黑盒，调度由 BVI 声明，不会撞上；所以这里照黑盒的
// 样子做：外面看到的每一根输出都来自寄存器。
//
// 0x08 读回 {读传输里选通没拉低的次数, 写过多少次}；0x0C 一律回错误。
module mkComp#(Integer waits)(Apb4SlavePins#(8, 32));
  Vector#(4, Reg#(Bit#(32))) v   <- replicateM(mkReg(0));
  Reg#(Bit#(16))             wc  <- mkReg(0);
  Reg#(Bit#(16))             sv  <- mkReg(0);   // 读传输里 PSTRB 没拉低的次数
  Reg#(Bit#(8))              cnt <- mkReg(0);
  Reg#(Bool)                 rdy <- mkReg(False);
  Reg#(Bit#(32))             rd  <- mkReg(0);
  Reg#(Bool)                 er  <- mkReg(False);

  Wire#(Bool)     acc <- mkDWire(False);
  Wire#(Bit#(8))  ad  <- mkDWire(0);
  Wire#(Bool)     wrW <- mkDWire(False);
  Wire#(Bit#(32)) wdW <- mkDWire(0);
  Wire#(Bit#(4))  stW <- mkDWire(0);

  rule step;
    Bit#(2) i = ad[3:2];
    if (!acc) begin
      cnt <= 0;
      rdy <= False;
    end else if (rdy) begin
      rdy <= False;
      cnt <= 0;
    end else if (cnt < fromInteger(waits))
      cnt <= cnt + 1;
    else begin
      // 下一拍抬 PREADY，同时把结果摆好——主机就在抬着的那一拍采样
      rdy <= True;
      rd  <= (ad == 8'h08) ? {sv, wc} : v[i];
      er  <= ad == 8'h0C;
      if (wrW && ad != 8'h0C) begin
        v[i] <= wdW;
        wc   <= wc + 1;
      end
      // 3.2：读传输里请求方必须把 PSTRB 全部拉低
      if (!wrW && stW != 0) sv <= sv + 1;
    end
  endrule

  method Action req(paddr, pprot, psel, penable, pwrite, pwdata, pstrb);
    acc <= psel && penable;
    ad  <= paddr;
    wrW <= pwrite;
    wdW <= pwdata;
    stW <= pstrb;
  endmethod
  method Bool     pready  = rdy;
  method Bit#(32) prdata  = rd;
  method Bool     pslverr = er;
endmodule

// 收编：第三方 APB4 从设备 -> 中立的会停顿目标。
(* synthesize *)
module mkAdoptTb(Empty);
  Apb4SlavePins#(8, 32) comp <- mkComp(2);
  RegTarget#(8, 32)     ad   <- mkApb4Adopt(comp);

  Reg#(Bit#(8))  ph   <- mkReg(0);
  Reg#(Bool)     hold <- mkReg(False);
  Reg#(RegReq#(8, 32)) q <- mkReg(unpack(0));
  Reg#(Bit#(16)) took <- mkReg(0);
  Reg#(Bool)     bad  <- mkReg(False);
  Reg#(Bit#(16)) cyc  <- mkReg(0);

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 2000) begin $display("TIMEOUT at phase %0d", ph); $finish(1); end
  endrule

  rule run;
    ad.req(hold, q);

    Bool     nh  = hold;
    Bit#(8)  np  = ph;
    Bool     nb  = bad;
    Bit#(16) nt  = took + 1;
    RegReq#(8, 32) nq = q;

    if (!hold) begin
      nt = 0;
      case (ph)
        0: begin nq = RegReq { addr: 8'h04, write: True,  wdata: 32'hADAD1234, wstrb: 4'hF }; nh = True; end
        1: begin nq = RegReq { addr: 8'h04, write: False, wdata: 0, wstrb: 4'hF }; nh = True; end
        2: begin nq = RegReq { addr: 8'h08, write: False, wdata: 0, wstrb: 4'hF }; nh = True; end
        3: begin nq = RegReq { addr: 8'h0C, write: False, wdata: 0, wstrb: 4'hF }; nh = True; end
        default: begin
                   if (bad) $display("FAILED");
                   else $display("PASS adopt: a third party APB4 completer answers "
                                 + "through the neutral stalling contract, one "
                                 + "transfer per request, data valid when it lands");
                   $finish(bad ? 1 : 0);
                 end
      endcase
    end else if (ad.rspValid) begin
      nh = False;
      np = ph + 1;
      case (ph)
        // SETUP 一拍、两拍等待、ACCESS 一拍，再加收编器登记答复的一拍：怎么也不止两拍。
        // 收编器要是发出请求的同一拍就回答，这一条当场就红。
        0: if (took < 3) begin
             $display("FAIL the adopter answered after %0d cycles, too fast for APB4", took);
             nb = True;
           end
        1: if (ad.rsp.rdata != 32'hADAD1234) begin
             $display("FAIL read back %08h, want ADAD1234", ad.rsp.rdata);
             nb = True;
           end
        // 低半是写次数（一笔请求只许一次），高半是读传输里选通没拉低的次数
        2: if (ad.rsp.rdata != 32'h00000001) begin
             $display("FAIL the completer saw %04h strobe violations and %04h writes",
                      ad.rsp.rdata[31:16], ad.rsp.rdata[15:0]);
             nb = True;
           end
        3: if (!ad.rsp.err) begin
             $display("FAIL PSLVERR did not come back through the contract");
             nb = True;
           end
      endcase
    end

    hold <= nh;
    ph   <= np;
    bad  <= nb;
    took <= nt;
    q    <= nq;
  endrule
endmodule

endpackage
