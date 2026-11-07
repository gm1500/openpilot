/* Optional contract check against a local commaai/connect checkout.
 * Run after test_session_recorder.py:
 * node test_connect_events.cjs /path/to/connect /path/to/connect-events.json
 * Uses Connect's actual event parser and colour selectors; no backend writes.
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const [checkout, fixture] = process.argv.slice(2);
assert(checkout && fixture, 'supply a Connect checkout and native recorder event fixture');
const cached = fs.readFileSync(path.join(checkout, 'src/actions/cached.js'), 'utf8');
const start = cached.indexOf('function parseEvents(route, driveEvents) {');
const end = cached.indexOf('export function fetchEvents', start);
assert(start >= 0 && end > start, 'review the updated Connect parser');
const parse = vm.runInNewContext(`(${cached.slice(start, end).trim()})`);
const input = JSON.parse(fs.readFileSync(fixture, 'utf8'));
const events = parse({duration: input.duration}, input.events);

const timeline = fs.readFileSync(path.join(checkout, 'src/components/Timeline/index.jsx'), 'utf8');
const theme = fs.readFileSync(path.join(checkout, 'src/theme.js'), 'utf8');
assert.match(timeline, /'&\.engage':\s*\{\s*background: theme\.palette\.states\.engagedGreen/);
assert.match(timeline, /'&\.critical':\s*\{\s*background: theme\.palette\.states\.alertRed/);
assert.match(timeline, /'&\.userPrompt':\s*\{\s*background: theme\.palette\.states\.alertOrange/);
assert.match(theme, /engagedGreen: Colors\.green400/);
assert.match(theme, /alertRed: Colors\.red100/);

function spans(type, status) {
  return Array.from(events.filter(e => e.type === type && (status === undefined || e.data.alertStatus === status)),
    e => [e.route_offset_millis / 1000, e.data.end_route_offset_millis / 1000]);
}
assert.deepEqual(spans('engage'), [[10, 11], [20, 22], [60, 62], [70, 72]]);
assert.deepEqual(spans('alert', 2), [[11, 13], [59, 60]]);
assert.deepEqual(spans('alert', 1), [[0, 10], [13, 20], [44, 59], [62, 70]]);
assert(spans('bookmark').length > 0, 'standard yellow bookmarks remain available');
console.log('Connect parser: green audio, red gyro, orange pre-roll and blue gap boundaries verified.');
