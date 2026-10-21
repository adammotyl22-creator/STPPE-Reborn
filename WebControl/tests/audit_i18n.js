import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { translations } from '../js/i18n/translations.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const rootDir = path.resolve(__dirname, '..');

const html = fs.readFileSync(path.join(rootDir, 'index.html'), 'utf8');

console.log('=== AUDITING I18N IN index.html ===');

// 1. Check title attributes without data-i18n-title
const titleRegex = /<[^>]*\btitle="([^"]+)"[^>]*>/g;
let match;
const missingTitle = [];
while ((match = titleRegex.exec(html)) !== null) {
  const tag = match[0];
  const title = match[1];
  if (!tag.includes('data-i18n-title') && !title.startsWith('{') && title.length > 2) {
    missingTitle.push({ tag: tag.slice(0, 100), title });
  }
}
console.log(`Titles without data-i18n-title: ${missingTitle.length}`);
missingTitle.forEach(m => console.log(`  - [${m.title}] in ${m.tag}`));

// 2. Check text nodes in elements without data-i18n
const textTagRegex = /<([a-zA-Z0-9_-]+)([^>]*)>([^<]+)<\/\1>/g;
const untranslatedText = [];
while ((match = textTagRegex.exec(html)) !== null) {
  const [full, tag, attrs, content] = match;
  const text = content.trim();
  if (text.length > 1 && !attrs.includes('data-i18n') && !text.match(/^[\d\.\-\+\:\%\s°±\/\|\(\)]+$/) && !attrs.includes('class="key-hint-badge"')) {
    untranslatedText.push({ tag, attrs: attrs.slice(0, 60), text });
  }
}
console.log(`\nUntranslated inner text elements count: ${untranslatedText.length}`);
untranslatedText.forEach(m => console.log(`  - [${m.text}] <${m.tag} ${m.attrs}>`));

