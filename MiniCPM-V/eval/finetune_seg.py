# -*- coding: utf-8 -*-
"""finetune_seg.py —— 在我们自己的窗框标注上微调 SegFormer

═══════════════════════════════════════════════════════════════════
为什么是"微调"而不是"换更大的模型"
───────────────────────────────────────────────────────────────────
现状（实测）：
  · CMP 预训练 SegFormer：域内（CMP）覆盖率 84.4%、纯度 89.1%、框级 0 假框
  · 换到我们宿舍楼（域外）：一张图出 50~59 个框，肉眼约 20~30 扇 → 过检约 2 倍

过检的根因是**训练分布不匹配**（欧洲街景立面 vs 中国宿舍楼 + 空调机位），
不是模型容量不够。所以：
    · 换 SegFormer-B5 / SAM2  → 域差还在，没用
    · 在**我们自己的框标注**上微调 → 直接改变训练分布 → 对症

═══════════════════════════════════════════════════════════════════
输入
───────────────────────────────────────────────────────────────────
  eval/labels_boxes/<图名>.json   ← 标注工具产出（窗框 + 原图尺寸）
  原图目录（--dir）

框标注会转成"软标签"掩膜：框内=1，框外=0。
这样做虽然不如逐像素标注精细，但对"窗在哪一大块"已经够用，
而且标注成本低 10 倍。

═══════════════════════════════════════════════════════════════════
策略（小样本必须这样训）
───────────────────────────────────────────────────────────────────
  · 冻结编码器前若干层，只训解码器 + 后几层
  · 学习率小（1e-5），轮数少（10~20）
  · 训练/验证按图划分（不能把同一张图既训又验）
  · 数据增强：水平翻转 + 亮度抖动

用法：
  python eval/finetune_seg.py --dir "D:\\Desktop\\window" --epochs 12
  python eval/finetune_seg.py --eval-only          # 只评估，不训练
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402

SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
OUTDIR = ROOT / "eval" / "models"
LB = ROOT / "eval" / "labels_boxes"


def load_pairs(img_dir=None):
    """返回 [(图路径, 框列表)]。

    以标注 JSON 里记录的 image 路径为准 —— 我们的图分散在
    data/samples 和 D:\\Desktop\\window 两个目录，不能只扫一个。
    """
    pairs = []
    for j in sorted(LB.glob("*.json")):
        try:
            d = json.loads(j.read_text(encoding="utf-8"))
        except Exception:
            continue
        p = Path(d.get("image", ""))
        if not p.exists():
            continue
        boxes = d.get("boxes") or []
        if boxes:
            pairs.append((p, boxes))
    return pairs


def boxes_to_mask(shape, boxes):
    m = np.zeros(shape[:2], np.uint8)
    H, W = shape[:2]
    for (x, y, w, h) in boxes:
        x0, y0 = max(0, int(x)), max(0, int(y))
        x1, y1 = min(W, int(x + w)), min(H, int(y + h))
        if x1 > x0 and y1 > y0:
            m[y0:y1, x0:x1] = 1
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=str(Path(r"D:\Desktop\window")))
    ap.add_argument("--epochs", type=int, default=12)
    ap.add_argument("--lr", type=float, default=1e-5)
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--eval-only", action="store_true")
    ap.add_argument("--loss", choices=["ce", "dice"], default="dice",
                    help="ce=纯交叉熵（偏保守）；dice=CE+Dice（提升覆盖率）")
    ap.add_argument("--suffix", default="", help="输出子目录后缀")
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    if not a.out:
        a.out = str(OUTDIR / ("seg_ours" + a.suffix))
    img_dir = Path(a.dir)
    pairs = load_pairs(img_dir)
    if not pairs:
        raise SystemExit(
            f"在 {img_dir} 下没找到有标注的图。\n"
            f"先双击 标注工具.cmd 标几张，结果会存到 {LB}")

    print(f"找到 {len(pairs)} 张有标注的图")
    if len(pairs) < 4:
        print("⚠ 少于 4 张，训练效果会很差；建议先标 10 张以上")

    import torch
    import torch.nn.functional as F
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    proc = AutoImageProcessor.from_pretrained(SEG_ID)
    model = AutoModelForSemanticSegmentation.from_pretrained(SEG_ID)
    wid = [i for i, n in model.config.id2label.items() if "window" in str(n).lower()][0]

    # 按图 8:2 划分
    n_val = max(1, int(len(pairs) * 0.2))
    val_pairs, tr_pairs = pairs[:n_val], pairs[n_val:]

    if not a.eval_only:
        dev = "cuda" if torch.cuda.is_available() else "cpu"
        model = model.to(dev)
        # 冻结编码器大部分参数
        for name, p in model.named_parameters():
            if name.startswith("segformer.encoder"):
                p.requires_grad = False
        trainable = [p for p in model.parameters() if p.requires_grad]
        opt = torch.optim.AdamW(trainable, lr=a.lr)
        print(f"训练 {len(tr_pairs)} 张 / 验证 {len(val_pairs)} 张，"
              f"设备 {dev}，可训参数 {sum(p.numel() for p in trainable)/1e6:.1f}M")

        for ep in range(a.epochs):
            model.train()
            tot = 0.0
            for f, boxes in tr_pairs:
                img = imread_unicode(f)
                if img is None:
                    continue
                pil = Image.fromarray(img[:, :, ::-1])
                inp = proc(images=pil, return_tensors="pt").to(dev)
                mask = boxes_to_mask(img.shape, boxes)
                gt = torch.from_numpy(mask).long().to(dev)
                out = model(**inp)
                # ★ 不要把 logits 上采样到 4096×3072（CPU 上极慢）；
                #   反过来把标注缩到模型输出尺度，等价且快几十倍。
                logits = out.logits                       # [1, C, h, w]
                tgt_full = torch.where(gt == 1, torch.full_like(gt, wid),
                                       torch.full_like(gt, 0))
                tgt = F.interpolate(tgt_full[None, None].float(),
                                    size=logits.shape[-2:],
                                    mode="nearest")[:, 0].long()
                ce = F.cross_entropy(logits, tgt)
                if a.loss == "dice":
                    # ★ Dice 直接优化重叠度，能治"只预测最有把握那一小块"的毛病
                    prob = torch.softmax(logits, 1)[:, wid]
                    tgtw = (tgt == wid).float()
                    num = 2 * (prob * tgtw).sum()
                    den = prob.sum() + tgtw.sum() + 1e-6
                    loss = ce + (1 - num / den)
                else:
                    loss = ce
                opt.zero_grad()
                loss.backward()
                opt.step()
                tot += float(loss)
            print(f"  epoch {ep+1}/{a.epochs}  loss {tot/max(1,len(tr_pairs)):.4f}")
            sys.stdout.flush()

        OUTDIR.mkdir(parents=True, exist_ok=True)
        model.save_pretrained(a.out)
        proc.save_pretrained(a.out)
        print(f"已保存 → {a.out}")
    else:
        from transformers import AutoModelForSemanticSegmentation as M
        model = M.from_pretrained(a.out).eval()

    # ── 评估：覆盖率 / 纯度（和之前同一套口径）─────────────
    print("\n评估（框级覆盖率与纯度，同一口径）")
    print(f"{'图':<14}{'框覆盖率':<10}{'框内纯度'}")
    covs, precs = [], []
    for f, boxes in val_pairs:
        img = imread_unicode(f)
        if img is None:
            continue
        pil = Image.fromarray(img[:, :, ::-1])
        with torch.no_grad():
            out = model(**proc(images=pil, return_tensors="pt"))
        logits = F.interpolate(out.logits, size=img.shape[:2],
                               mode="nearest")
        pred = (logits.argmax(1)[0].numpy() == wid).astype(np.uint8)
        gt = boxes_to_mask(img.shape, boxes)
        inter = int((pred & gt).sum())
        cov = inter / max(1, int(gt.sum()))
        prec = inter / max(1, int(pred.sum()))
        covs.append(cov)
        precs.append(prec)
        print(f"  {f.stem:<12}{cov*100:5.1f}%    {prec*100:5.1f}%")
    if covs:
        print(f"\n  平均覆盖率 {100*np.mean(covs):.1f}%   平均纯度 {100*np.mean(precs):.1f}%")


if __name__ == "__main__":
    main()
