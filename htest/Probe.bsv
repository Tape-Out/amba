package Probe;

import AmbaCfg::*;

import RegIf::*;
import Apb4::*;

// 量总线绑定器用的探针。绑定器对位宽是多态的，综合得先把它钉在一个具体位宽上，
// 桩取纯组合，好让量到的数尽量只含绑定器自己。
//
// 探针存在的理由是**手工量过的数字必须留下配方**：`amba` 的 90.44 量过一次就
// 再没人能重现，源码一改摘要就过期，而过期之后没有任何命令能把它测回来。
(* synthesize *)
module mkApb4BindProbe(Apb4SlavePins#(AW, DW));
  RegIf#(AW, DW) stub = interface RegIf;
      method ActionValue#(RegRsp#(DW)) access(RegReq#(AW, DW) r);
        return RegRsp { rdata: zeroExtend(r.addr), err: False };
      endmethod
    endinterface;
  let p <- mkApb4Bind(stub);
  return p;
endmodule

endpackage
