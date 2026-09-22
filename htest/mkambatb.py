"""amba 的测试台配置：把这一点的位宽发成 BSV 的数值类型。

`Apb4` 从头到尾按 `aw`/`dw` 写，测试台却钉死在 8/32 —— 于是整个总线库只在
一种位宽下测过，而位宽算术恰恰是最容易错的地方。这里只发两个 typedef，
测试台里的数据宽度、地址宽度、写选通宽度全都从它们算出来。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
k = (json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}).get("knobs", {})
aw = int(k.get("aw", 8))
dw = int(k.get("dw", 32))

(out / "AmbaCfg.bsv").write_text(
    "package AmbaCfg;\n\n"
    f"typedef {aw} AW;\n"
    f"typedef {dw} DW;\n\n"
    "endpackage\n", encoding="utf-8")
