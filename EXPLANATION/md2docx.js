// Minimal Markdown -> .docx renderer for the SensoSpoon progress reports.
// Handles exactly what these reports use: headings, paragraphs, bullet and
// numbered lists, pipe tables, fenced code blocks, horizontal rules, and the
// inline **bold** / *italic* / `code` forms.
const fs = require('fs');
const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType,
  Table, TableRow, TableCell, WidthType, BorderStyle, ShadingType,
  LevelFormat, PageOrientation,
} = require('docx');

const SRC = process.argv[2];
const OUT = process.argv[3];

const CONTENT_W = 9360;           // US Letter 12240 less 1440 margins each side
const BODY = 'Calibri';
const MONO = 'Consolas';
const INK = '1A1A1A';
const INK2 = '4A4A48';
const RULE = 'BFBFBF';
const HEADBG = 'EDF2F7';

// ---------------------------------------------------------------- inline runs
function runs(text, opts = {}) {
  const base = { font: opts.font || BODY, size: opts.size || 21, color: opts.color || INK };
  const out = [];
  // Recursive, because these reports genuinely nest the forms -- **`ekf_cv_semg`**
  // is bold-wrapped inline code, and a single non-recursive pass renders the
  // backticks literally.
  const walk = (s, style) => {
    const re = /(`[^`]+`|\*\*[^*`]*(?:`[^`]+`[^*`]*)*\*\*|\*[^*\s][^*]*\*)/g;
    let last = 0, m;
    const push = (t, extra) => { if (t) out.push(new TextRun({ ...base, ...style, ...extra, text: t })); };
    while ((m = re.exec(s)) !== null) {
      push(s.slice(last, m.index), {});
      const tok = m[0];
      if (tok.startsWith('`')) {
        push(tok.slice(1, -1), { font: MONO, size: base.size - 2, color: style.bold ? '0B5A66' : '0B5A66' });
      } else if (tok.startsWith('**')) {
        walk(tok.slice(2, -2), { ...style, bold: true });
      } else {
        walk(tok.slice(1, -1), { ...style, italics: true });
      }
      last = m.index + tok.length;
    }
    push(s.slice(last), {});
  };
  walk(text, {});
  return out.length ? out : [new TextRun({ ...base, text: '' })];
}

function stripInline(t) {
  return t.replace(/`([^`]+)`/g, '$1').replace(/\*\*([^*]+)\*\*/g, '$1').replace(/\*([^*]+)\*/g, '$1');
}

// ---------------------------------------------------------------- table build
function buildTable(header, aligns, rows) {
  const ncol = header.length;
  // Proportional widths from the longest cell in each column, clamped so one
  // long column cannot starve the rest.
  const raw = header.map((h, i) => {
    let w = stripInline(h).length;
    rows.forEach(r => { w = Math.max(w, stripInline(r[i] || '').length); });
    return Math.min(Math.max(w, 6), 46);
  });
  const total = raw.reduce((a, b) => a + b, 0);
  let widths = raw.map(w => Math.floor(CONTENT_W * w / total));
  widths[ncol - 1] += CONTENT_W - widths.reduce((a, b) => a + b, 0);

  const cellBorders = {
    top: { style: BorderStyle.SINGLE, size: 2, color: RULE },
    bottom: { style: BorderStyle.SINGLE, size: 2, color: RULE },
    left: { style: BorderStyle.NONE },
    right: { style: BorderStyle.NONE },
  };
  const align = i => (aligns[i] === 'right' ? AlignmentType.RIGHT
    : aligns[i] === 'center' ? AlignmentType.CENTER : AlignmentType.LEFT);

  const mkCell = (text, i, isHead) => new TableCell({
    width: { size: widths[i], type: WidthType.DXA },
    borders: cellBorders,
    shading: isHead ? { type: ShadingType.CLEAR, fill: HEADBG, color: 'auto' } : undefined,
    margins: { top: 60, bottom: 60, left: 110, right: 110 },
    children: [new Paragraph({
      alignment: align(i),
      spacing: { before: 0, after: 0 },
      children: runs(text, { size: 19, color: isHead ? INK : INK }).map(r => r),
    })],
  });

  return new Table({
    columnWidths: widths,
    width: { size: CONTENT_W, type: WidthType.DXA },
    rows: [
      new TableRow({
        tableHeader: true,
        children: header.map((h, i) => mkCell(h.replace(/\*\*/g, ''), i, true)),
      }),
      ...rows.map(r => new TableRow({
        children: r.map((c, i) => mkCell(c, i, false)),
      })),
    ],
  });
}

function splitRow(line) {
  return line.replace(/^\s*\|/, '').replace(/\|\s*$/, '').split('|').map(s => s.trim());
}

// ---------------------------------------------------------------- main parse
const lines = fs.readFileSync(SRC, 'utf8').split(/\r?\n/);
const children = [];
let i = 0;

const para = (text, o = {}) => new Paragraph({
  spacing: { before: o.before ?? 60, after: o.after ?? 140, line: 276 },
  alignment: o.alignment,
  children: runs(text, o),
});

while (i < lines.length) {
  const line = lines[i];

  if (/^\s*$/.test(line)) { i++; continue; }

  // fenced code
  if (/^```/.test(line)) {
    i++;
    const code = [];
    while (i < lines.length && !/^```/.test(lines[i])) { code.push(lines[i]); i++; }
    i++;
    code.forEach((c, k) => children.push(new Paragraph({
      spacing: { before: k === 0 ? 100 : 0, after: k === code.length - 1 ? 160 : 0 },
      shading: { type: ShadingType.CLEAR, fill: 'F4F5F7', color: 'auto' },
      indent: { left: 220, right: 220 },
      children: [new TextRun({ font: MONO, size: 18, color: '243447', text: c || ' ' })],
    })));
    continue;
  }

  // horizontal rule -> a thin bottom border on an empty paragraph
  if (/^\s*---+\s*$/.test(line)) {
    children.push(new Paragraph({
      spacing: { before: 120, after: 200 },
      border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: RULE, space: 6 } },
      children: [new TextRun({ text: '' })],
    }));
    i++; continue;
  }

  // table
  if (/^\s*\|/.test(line) && i + 1 < lines.length && /^\s*\|[\s:\-|]+\|\s*$/.test(lines[i + 1])) {
    const header = splitRow(line);
    const aligns = splitRow(lines[i + 1]).map(s =>
      /^:-+:$/.test(s) ? 'center' : /-+:$/.test(s) ? 'right' : 'left');
    i += 2;
    const rows = [];
    while (i < lines.length && /^\s*\|/.test(lines[i])) { rows.push(splitRow(lines[i])); i++; }
    children.push(buildTable(header, aligns, rows));
    children.push(new Paragraph({ spacing: { before: 0, after: 200 }, children: [new TextRun('')] }));
    continue;
  }

  // headings
  const h = /^(#{1,4})\s+(.*)$/.exec(line);
  if (h) {
    const depth = h[1].length;
    const text = h[2].trim();
    if (depth === 1) {
      children.push(new Paragraph({
        heading: HeadingLevel.TITLE,
        spacing: { before: 0, after: 60 },
        children: runs(text, { size: 40, color: INK }).map(r => r),
      }));
    } else if (depth === 2) {
      children.push(new Paragraph({
        heading: HeadingLevel.HEADING_1,
        spacing: { before: 320, after: 120 },
        border: { bottom: { style: BorderStyle.SINGLE, size: 4, color: RULE, space: 4 } },
        children: runs(text, { size: 27, color: INK }).map(r => r),
      }));
    } else {
      children.push(new Paragraph({
        heading: HeadingLevel.HEADING_2,
        spacing: { before: 240, after: 100 },
        children: runs(text, { size: 23, color: INK }).map(r => r),
      }));
    }
    i++; continue;
  }

  // bullet list
  if (/^\s*-\s+/.test(line)) {
    while (i < lines.length && /^\s*-\s+/.test(lines[i])) {
      let text = lines[i].replace(/^\s*-\s+/, '');
      i++;
      while (i < lines.length && /^\s{2,}\S/.test(lines[i]) && !/^\s*-\s+/.test(lines[i])) {
        text += ' ' + lines[i].trim(); i++;
      }
      children.push(new Paragraph({
        numbering: { reference: 'bullets', level: 0 },
        spacing: { before: 40, after: 80, line: 276 },
        children: runs(text),
      }));
    }
    continue;
  }

  // numbered list
  if (/^\s*\d+\.\s+/.test(line)) {
    while (i < lines.length && /^\s*\d+\.\s+/.test(lines[i])) {
      let text = lines[i].replace(/^\s*\d+\.\s+/, '');
      i++;
      while (i < lines.length && /^\s{3,}\S/.test(lines[i]) && !/^\s*\d+\.\s+/.test(lines[i])) {
        text += ' ' + lines[i].trim(); i++;
      }
      children.push(new Paragraph({
        numbering: { reference: 'numbers', level: 0 },
        spacing: { before: 40, after: 80, line: 276 },
        children: runs(text),
      }));
    }
    continue;
  }

  // paragraph (join wrapped lines)
  let text = line.trim();
  i++;
  while (i < lines.length && !/^\s*$/.test(lines[i]) && !/^\s*[-#|>]/.test(lines[i])
         && !/^```/.test(lines[i]) && !/^\s*\d+\.\s+/.test(lines[i])) {
    text += ' ' + lines[i].trim(); i++;
  }
  children.push(para(text));
}

const doc = new Document({
  creator: 'SensoSpoon project',
  title: 'Control Improvement Progress Report V3',
  description: 'sEMG + IMU fusion, and the final benchmark',
  numbering: {
    config: [
      {
        reference: 'bullets',
        levels: [{
          level: 0, format: LevelFormat.BULLET, text: '\u2022',
          alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 460, hanging: 260 } } },
        }],
      },
      {
        reference: 'numbers',
        levels: [{
          level: 0, format: LevelFormat.DECIMAL, text: '%1.',
          alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 460, hanging: 260 } } },
        }],
      },
    ],
  },
  styles: {
    default: {
      document: { run: { font: BODY, size: 21, color: INK } },
    },
  },
  sections: [{
    properties: {
      page: {
        size: { width: 12240, height: 15840, orientation: PageOrientation.PORTRAIT },
        margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 },
      },
    },
    children,
  }],
});

Packer.toBuffer(doc).then(buf => {
  fs.writeFileSync(OUT, buf);
  console.log('wrote ' + OUT + ' (' + buf.length + ' bytes)');
});
