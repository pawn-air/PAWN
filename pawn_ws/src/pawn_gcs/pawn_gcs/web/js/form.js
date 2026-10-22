// Renders a parameter form from a plugin/controller `params` spec.
import { h } from './core.js';

export function paramForm(params, values, onChange) {
  const box = h('div', { class: 'form' });
  for (const [k, spec] of Object.entries(params || {})) {
    const v = values?.[k] ?? spec.default;
    let input;
    const fire = (val) => onChange(k, val);
    if (spec.type === 'bool') {
      input = h('input', { type: 'checkbox', checked: !!v, onchange: (e) => fire(e.target.checked) });
    } else if (spec.type === 'select') {
      input = h('select', { onchange: (e) => fire(e.target.value) }, ...(spec.options || []).map(o => h('option', { value: o, selected: o === v }, o)));
    } else if ((spec.type === 'int' || spec.type === 'float') && spec.min != null && spec.max != null) {
      const out = h('output', {}, v);
      const step = spec.step ?? (spec.type === 'int' ? 1 : (spec.max - spec.min) / 100);
      input = h('div', { class: 'val' }, h('input', {
        type: 'range', min: spec.min, max: spec.max, step, value: v,
        oninput: (e) => { out.value = e.target.value; }, onchange: (e) => fire(Number(e.target.value)),
      }), out);
    } else if (spec.type === 'int' || spec.type === 'float') {
      input = h('input', { type: 'number', value: v, step: spec.step ?? 'any', onchange: (e) => fire(Number(e.target.value)) });
    } else {
      input = h('input', { value: v ?? '', onchange: (e) => fire(e.target.value) });
    }
    box.append(h('label', { title: k }, spec.label || k), input);
  }
  return box;
}
