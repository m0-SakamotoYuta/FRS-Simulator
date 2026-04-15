const pptxgen = require("pptxgenjs");

let pres = new pptxgen();
pres.layout = 'LAYOUT_16x9';
pres.title = '3Dモデル位置合わせの仕組み';

// Color palette
const C = {
  navy:     "1A2E4A",
  blue:     "1D5FA5",
  lightblue:"D6E8FA",
  teal:     "0D9488",
  white:    "FFFFFF",
  offwhite: "F5F8FC",
  dark:     "1E293B",
  mid:      "475569",
  light:    "94A3B8",
  gold:     "F59E0B",
  green:    "16A34A",
  red:      "DC2626",
};

const FONT = "Meiryo";
const SLIDE_W = 10, SLIDE_H = 5.625;

function addSlideNumber(slide, num) {
  slide.addText(`${num} / 7`, {
    x: 9.3, y: 5.25, w: 0.6, h: 0.25,
    fontSize: 10, color: C.light, align: "right", fontFace: FONT
  });
}

function addHeader(slide, title, color) {
  // colored top bar
  slide.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 0, w: SLIDE_W, h: 1.0,
    fill: { color: color || C.navy }, line: { color: color || C.navy }
  });
  slide.addText(title, {
    x: 0.5, y: 0.1, w: 9, h: 0.8,
    fontSize: 26, bold: true, color: C.white,
    fontFace: FONT, valign: "middle", margin: 0
  });
}

// =============================================
// SLIDE 1: Title
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.navy };

  // Top accent bar
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 0, w: SLIDE_W, h: 0.12,
    fill: { color: C.teal }, line: { color: C.teal }
  });
  // Bottom accent bar
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 5.5, w: SLIDE_W, h: 0.125,
    fill: { color: C.teal }, line: { color: C.teal }
  });

  // Decorative circle
  s.addShape(pres.shapes.OVAL, {
    x: 7.5, y: 0.8, w: 3.5, h: 3.5,
    fill: { color: C.blue, transparency: 70 },
    line: { color: C.blue, transparency: 70 }
  });

  s.addText("3Dモデル位置合わせの仕組み", {
    x: 0.6, y: 1.5, w: 7, h: 1.2,
    fontSize: 34, bold: true, color: C.white,
    fontFace: FONT, valign: "middle"
  });
  s.addText("RANSAC・ICP とスケール補正", {
    x: 0.6, y: 2.9, w: 7, h: 0.7,
    fontSize: 20, color: C.lightblue,
    fontFace: FONT, valign: "middle"
  });

  addSlideNumber(s, 1);
}

// =============================================
// SLIDE 2: 処理の全体フロー
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.offwhite };
  addHeader(s, "処理の全体フロー", C.navy);

  const steps = [
    { num: "1", label: "主軸アライメント\n（PCA）", desc: "重心と主軸方向を\n一致させる前処理", color: C.teal },
    { num: "2", label: "RANSAC", desc: "特徴点マッチングによる\n粗い位置合わせ", color: C.blue },
    { num: "3", label: "ICP", desc: "点群全体を使った\n精密な位置合わせ\n・スケール補正", color: C.navy },
  ];

  const boxW = 2.4, boxH = 2.8, startX = 0.8, y = 1.3, gap = 0.9;

  steps.forEach((step, i) => {
    const x = startX + i * (boxW + gap);

    // Card background
    s.addShape(pres.shapes.RECTANGLE, {
      x, y, w: boxW, h: boxH,
      fill: { color: C.white },
      line: { color: step.color, width: 2 },
      shadow: { type: "outer", color: "000000", blur: 6, offset: 3, angle: 135, opacity: 0.1 }
    });
    // Colored top
    s.addShape(pres.shapes.RECTANGLE, {
      x, y, w: boxW, h: 0.55,
      fill: { color: step.color },
      line: { color: step.color }
    });
    // Step number circle
    s.addShape(pres.shapes.OVAL, {
      x: x + 0.05, y: y + 0.05, w: 0.44, h: 0.44,
      fill: { color: C.white },
      line: { color: C.white }
    });
    s.addText(step.num, {
      x: x + 0.05, y: y + 0.05, w: 0.44, h: 0.44,
      fontSize: 16, bold: true, color: step.color,
      fontFace: FONT, align: "center", valign: "middle", margin: 0
    });
    // Step label
    s.addText(step.label, {
      x, y: y + 0.55, w: boxW, h: 0.9,
      fontSize: 15, bold: true, color: C.dark,
      fontFace: FONT, align: "center", valign: "middle"
    });
    // Description
    s.addText(step.desc, {
      x: x + 0.1, y: y + 1.5, w: boxW - 0.2, h: 1.2,
      fontSize: 12, color: C.mid,
      fontFace: FONT, align: "center", valign: "top"
    });

    // Arrow between boxes
    if (i < steps.length - 1) {
      const arrowX = x + boxW + 0.1;
      s.addShape(pres.shapes.LINE, {
        x: arrowX, y: y + boxH / 2, w: gap - 0.2, h: 0,
        line: { color: C.mid, width: 2 }
      });
      s.addText("▶", {
        x: arrowX + gap - 0.45, y: y + boxH / 2 - 0.18, w: 0.35, h: 0.35,
        fontSize: 16, color: C.mid, fontFace: FONT, align: "center", margin: 0
      });
    }
  });

  addSlideNumber(s, 2);
}

// =============================================
// SLIDE 3: RANSACの仕組み
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.offwhite };
  addHeader(s, "RANSAC（粗い位置合わせ）", C.blue);

  // Full name badge
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 1.1, w: 5.2, h: 0.4,
    fill: { color: C.lightblue },
    line: { color: C.blue }
  });
  s.addText("Random Sample Consensus", {
    x: 0.5, y: 1.1, w: 5.2, h: 0.4,
    fontSize: 12, color: C.blue, bold: true,
    fontFace: FONT, align: "center", valign: "middle", margin: 0
  });

  // Steps
  const steps = [
    { n: "1", t: "特徴量計算", d: "点群から FPFH（Fast Point Feature Histograms）を計算" },
    { n: "2", t: "ランダムサンプリング", d: "特徴量が類似する点のペアをランダムに選択" },
    { n: "3", t: "変換行列の推定", d: "選択したペアから回転・並進・スケールを計算" },
    { n: "4", t: "Inlier評価", d: "全点との対応数（Inlier数）でモデルを評価" },
    { n: "5", t: "最良解を採用", d: "最も多くの Inlier を持つ変換を最終解とする" },
  ];

  steps.forEach((step, i) => {
    const y = 1.65 + i * 0.7;
    s.addShape(pres.shapes.OVAL, {
      x: 0.5, y: y, w: 0.42, h: 0.42,
      fill: { color: C.blue },
      line: { color: C.blue }
    });
    s.addText(step.n, {
      x: 0.5, y: y, w: 0.42, h: 0.42,
      fontSize: 13, bold: true, color: C.white,
      fontFace: FONT, align: "center", valign: "middle", margin: 0
    });
    s.addText(step.t, {
      x: 1.1, y: y, w: 1.8, h: 0.42,
      fontSize: 13, bold: true, color: C.dark,
      fontFace: FONT, valign: "middle"
    });
    s.addText(step.d, {
      x: 3.0, y: y, w: 4.5, h: 0.42,
      fontSize: 12, color: C.mid,
      fontFace: FONT, valign: "middle"
    });
  });

  // Feature box (right)
  s.addShape(pres.shapes.RECTANGLE, {
    x: 7.7, y: 1.1, w: 2.1, h: 3.5,
    fill: { color: C.lightblue },
    line: { color: C.blue }
  });
  s.addText("特徴", {
    x: 7.7, y: 1.1, w: 2.1, h: 0.4,
    fontSize: 13, bold: true, color: C.blue,
    fontFace: FONT, align: "center", valign: "middle"
  });
  s.addText([
    { text: "✓ 外れ値（誤対応）に強い", options: { breakLine: true } },
    { text: "✓ 初期位置によらない大域的探索が可能", options: { breakLine: true } },
    { text: "✓ 大まかな姿勢の初期値を提供", options: {} },
  ], {
    x: 7.8, y: 1.55, w: 1.9, h: 3.0,
    fontSize: 11, color: C.dark,
    fontFace: FONT, valign: "top"
  });

  addSlideNumber(s, 3);
}

// =============================================
// SLIDE 4: ICPの仕組み
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.offwhite };
  addHeader(s, "ICP（精密な位置合わせ）", C.navy);

  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 1.1, w: 4.5, h: 0.4,
    fill: { color: C.lightblue },
    line: { color: C.navy }
  });
  s.addText("Iterative Closest Point", {
    x: 0.5, y: 1.1, w: 4.5, h: 0.4,
    fontSize: 12, color: C.navy, bold: true,
    fontFace: FONT, align: "center", valign: "middle", margin: 0
  });

  const steps = [
    { n: "1", t: "最近傍点の対応付け", d: "各点に対して相手点群の最も近い点を対応付け" },
    { n: "2", t: "変換行列の計算", d: "対応点間の距離を最小化する変換（回転・並進・スケール）を計算" },
    { n: "3", t: "変換の適用", d: "計算した変換行列を点群に適用" },
    { n: "4", t: "収束まで繰り返す", d: "変化量が閾値を下回るか、最大反復回数に達するまで1〜3を繰り返す" },
  ];

  steps.forEach((step, i) => {
    const y = 1.65 + i * 0.78;
    s.addShape(pres.shapes.OVAL, {
      x: 0.5, y: y, w: 0.42, h: 0.42,
      fill: { color: C.navy },
      line: { color: C.navy }
    });
    s.addText(step.n, {
      x: 0.5, y: y, w: 0.42, h: 0.42,
      fontSize: 13, bold: true, color: C.white,
      fontFace: FONT, align: "center", valign: "middle", margin: 0
    });
    s.addText(step.t, {
      x: 1.1, y: y, w: 2.2, h: 0.42,
      fontSize: 13, bold: true, color: C.dark,
      fontFace: FONT, valign: "middle"
    });
    s.addText(step.d, {
      x: 3.5, y: y, w: 4.0, h: 0.42,
      fontSize: 12, color: C.mid,
      fontFace: FONT, valign: "middle"
    });
  });

  // Feature box
  s.addShape(pres.shapes.RECTANGLE, {
    x: 7.7, y: 1.1, w: 2.1, h: 3.5,
    fill: { color: C.lightblue },
    line: { color: C.navy }
  });
  s.addText("特徴", {
    x: 7.7, y: 1.1, w: 2.1, h: 0.4,
    fontSize: 13, bold: true, color: C.navy,
    fontFace: FONT, align: "center", valign: "middle"
  });
  s.addText([
    { text: "✓ 高精度なスケール補正", options: { breakLine: true } },
    { text: "✓ 点群全体を使った局所最適化", options: { breakLine: true } },
    { text: "✓ RANSACの初期値を活かして高精度化", options: {} },
  ], {
    x: 7.8, y: 1.55, w: 1.9, h: 3.0,
    fontSize: 11, color: C.dark,
    fontFace: FONT, valign: "top"
  });

  addSlideNumber(s, 4);
}

// =============================================
// SLIDE 5: なぜRANSACではスケール補正が難しいか
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.offwhite };
  addHeader(s, "RANSACでスケール補正が難しい理由", C.blue);

  const reasons = [
    {
      num: "①",
      title: "特徴点の対応付けが前提",
      points: [
        "FPFH特徴量の類似度で点を対応付ける",
        "スケール差が大きいと同じ部位でも特徴量が変化",
        "正しい対応が取れず、スケール推定が不正確になる",
      ],
      color: C.blue,
    },
    {
      num: "②",
      title: "ダウンサンプリングによる点群の粗さ",
      points: [
        "計算速度のため点群を間引いて使用",
        "疎な点群ではスケール変化を検出しにくい",
        "細かい形状の違いが失われる",
      ],
      color: C.teal,
    },
    {
      num: "③",
      title: "変換の自由度と最適化の難しさ",
      points: [
        "スケールを加えると変換行列の自由度が増加",
        "ランダムサンプリングで正しいスケールを引き当てるのが困難",
        "外れ値が多いとスケール推定がぶれやすい",
      ],
      color: C.navy,
    },
  ];

  const cardW = 2.9, cardH = 3.2, startX = 0.35, cardY = 1.15, gap = 0.2;

  reasons.forEach((r, i) => {
    const x = startX + i * (cardW + gap);
    s.addShape(pres.shapes.RECTANGLE, {
      x, y: cardY, w: cardW, h: cardH,
      fill: { color: C.white },
      line: { color: r.color, width: 2 },
      shadow: { type: "outer", color: "000000", blur: 5, offset: 3, angle: 135, opacity: 0.1 }
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x, y: cardY, w: cardW, h: 0.55,
      fill: { color: r.color },
      line: { color: r.color }
    });
    s.addText(`理由 ${r.num}  ${r.title}`, {
      x: x + 0.1, y: cardY, w: cardW - 0.2, h: 0.55,
      fontSize: 12, bold: true, color: C.white,
      fontFace: FONT, valign: "middle"
    });
    r.points.forEach((pt, j) => {
      s.addText(`・ ${pt}`, {
        x: x + 0.15, y: cardY + 0.65 + j * 0.8, w: cardW - 0.3, h: 0.75,
        fontSize: 12, color: C.dark,
        fontFace: FONT, valign: "top"
      });
    });
  });

  // Conclusion bar
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.35, y: 4.55, w: 9.3, h: 0.65,
    fill: { color: C.navy },
    line: { color: C.navy }
  });
  s.addText("結論：RANSACの役割は「大まかな姿勢の初期値を与えること」。精密なスケール補正はICPが担う。", {
    x: 0.5, y: 4.55, w: 9.0, h: 0.65,
    fontSize: 13, bold: true, color: C.white,
    fontFace: FONT, valign: "middle", align: "center"
  });

  addSlideNumber(s, 5);
}

// =============================================
// SLIDE 6: 実際の処理結果の解釈
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.offwhite };
  addHeader(s, "実際の処理結果の解釈", C.navy);

  const stages = [
    { label: "処理前", color1: "2563EB", color2: "DC2626", desc: "2つのモデルが\n離れた位置にある", ok: false },
    { label: "RANSAC後", color1: "DC2626", color2: "EAB308", desc: "形状の重なりは改善\nスケール差が残る", ok: true },
    { label: "ICP後", color1: "DC2626", color2: "16A34A", desc: "スケールを含めて\n精密に一致", ok: true },
  ];

  stages.forEach((st, i) => {
    const x = 0.5 + i * 3.2;
    const y = 1.1;

    // Stage label
    s.addShape(pres.shapes.RECTANGLE, {
      x, y, w: 2.8, h: 0.38,
      fill: { color: C.navy },
      line: { color: C.navy }
    });
    s.addText(st.label, {
      x, y, w: 2.8, h: 0.38,
      fontSize: 14, bold: true, color: C.white,
      fontFace: FONT, align: "center", valign: "middle", margin: 0
    });

    // Visual mock (two overlapping circles)
    const cx1 = x + 0.85, cx2 = x + 1.45;
    const cy = y + 1.1;
    const r = 0.55;
    const offset = (i === 0) ? 0.45 : 0.0;

    s.addShape(pres.shapes.OVAL, {
      x: cx1 - offset - r, y: cy - r, w: r * 2, h: r * 2,
      fill: { color: st.color1, transparency: 30 },
      line: { color: st.color1 }
    });
    s.addShape(pres.shapes.OVAL, {
      x: cx2 + offset - r, y: cy - r + (i === 0 ? 0.3 : 0), w: r * 2, h: r * 2,
      fill: { color: st.color2, transparency: 30 },
      line: { color: st.color2 }
    });

    // Description
    s.addText(st.desc, {
      x, y: y + 2.4, w: 2.8, h: 0.9,
      fontSize: 12, color: C.dark,
      fontFace: FONT, align: "center", valign: "top"
    });
  });

  // Arrows between stages
  [1, 2].forEach(i => {
    const ax = 0.5 + i * 3.2 - 0.35;
    s.addText("▶", {
      x: ax, y: 2.0, w: 0.3, h: 0.35,
      fontSize: 18, color: C.mid,
      fontFace: FONT, align: "center", margin: 0
    });
  });

  // Summary
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 4.7, w: 9.0, h: 0.5,
    fill: { color: C.lightblue },
    line: { color: C.blue }
  });
  s.addText("→ RANSACは「位置合わせの道筋をつける役割」として正常に機能している", {
    x: 0.6, y: 4.7, w: 8.8, h: 0.5,
    fontSize: 13, bold: true, color: C.navy,
    fontFace: FONT, valign: "middle"
  });

  addSlideNumber(s, 6);
}

// =============================================
// SLIDE 7: まとめ
// =============================================
{
  let s = pres.addSlide();
  s.background = { color: C.navy };

  // Top accent
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 0, w: SLIDE_W, h: 0.12,
    fill: { color: C.teal }, line: { color: C.teal }
  });

  s.addText("まとめ", {
    x: 0.5, y: 0.25, w: 9, h: 0.7,
    fontSize: 28, bold: true, color: C.white,
    fontFace: FONT
  });

  const items = [
    {
      label: "RANSAC",
      color: C.blue,
      text: "特徴点ベースの大域的探索。外れ値に強く初期位置によらないが、スケール補正の精度は限定的。",
    },
    {
      label: "ICP",
      color: C.teal,
      text: "最近傍点ベースの局所最適化。RANSACの初期値があることで高精度なスケール補正を実現。",
    },
    {
      label: "2段階の組み合わせ",
      color: C.gold,
      text: "RANSACで粗く合わせ → ICPで精密に仕上げることで、スケール差のある3Dモデルの高精度な位置合わせを実現。",
    },
  ];

  items.forEach((item, i) => {
    const y = 1.15 + i * 1.25;
    // Left badge
    s.addShape(pres.shapes.RECTANGLE, {
      x: 0.5, y, w: 2.4, h: 0.9,
      fill: { color: item.color },
      line: { color: item.color }
    });
    s.addText(item.label, {
      x: 0.5, y, w: 2.4, h: 0.9,
      fontSize: 15, bold: true, color: C.white,
      fontFace: FONT, align: "center", valign: "middle", margin: 0
    });
    // Content card
    s.addShape(pres.shapes.RECTANGLE, {
      x: 3.1, y, w: 6.5, h: 0.9,
      fill: { color: "1E3A5F" },
      line: { color: item.color, width: 1 }
    });
    s.addText(item.text, {
      x: 3.2, y, w: 6.3, h: 0.9,
      fontSize: 13, color: C.lightblue,
      fontFace: FONT, valign: "middle"
    });
  });

  // Bottom accent
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 5.5, w: SLIDE_W, h: 0.125,
    fill: { color: C.teal }, line: { color: C.teal }
  });

  addSlideNumber(s, 7);
}

// Save
pres.writeFile({ fileName: "C:/Users/bio/Desktop/python/RANSAC_ICP_説明資料.pptx" })
  .then(() => console.log("✅ 保存完了: RANSAC_ICP_説明資料.pptx"))
  .catch(e => console.error("❌ エラー:", e));
