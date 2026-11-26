/* Optional integration check using Connect's exact hls.js bundle. No network,
 * credentials, device or backend changes. Run after test_session_recorder.py:
 * node test_connect_preview.cjs /path/to/hls-1.4.8.js /path/to/connect-preview
 * Decodable stream.mp4 and seek-N.mp4 are written beside the TS test fixtures.
 * This checks Connect's transmuxer, not native iPhone playback or its server.
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const [bundle, directory] = process.argv.slice(2);
assert(bundle && directory, 'supply hls.js 1.4.8 and the native test connect-preview directory');
const expose = '  return Hls;\n\n}));';
const source = fs.readFileSync(bundle, 'utf8');
assert(source.includes(expose), 'unexpected hls.js bundle layout');
global.self = global;
Object.defineProperty(global, 'navigator', {value: {userAgent: 'Mozilla/5.0 Chrome/140.0.0.0', vendor: 'Google Inc.'}, configurable: true});
vm.runInThisContext(source.replace(expose, '  Hls.Transmuxer = Transmuxer; return Hls;\n\n}));'), {filename: bundle});
assert.equal(Hls.version, '1.4.8', 'update this check only after reviewing Connect DriveVideo');

function check(order, seeking) {
  const errors = [];
  const config = {...Hls.DefaultConfig, maxBufferLength: 40, progressive: false};
  const mux = new Hls.Transmuxer({emit: (...event) => errors.push(event)}, {mp4: true, mpeg: false, mp3: true}, config, 'Google Inc.', 'main');
  let initialization;
  const stream = [];
  for (const [position, number] of order.entries()) {
    mux.configure({duration: 240, defaultInitPts: null});
    const meta = {sn: number, part: -1, level: 0, id: position, transmuxing: {}};
    const state = {contiguous: position > 0 && !seeking, discontinuity: position === 0, trackSwitch: position === 0,
      accurateTimeOffset: true, timeOffset: number*60, initSegmentChange: position === 0};
    const bytes = new Uint8Array(fs.readFileSync(path.join(directory, `${number}.ts`)));
    const {remuxResult: result} = mux.push(bytes, null, meta, state);
    if (result.initSegment?.tracks.video) initialization = Buffer.from(result.initSegment.tracks.video.initSegment);
    const video = result.video;
    assert(video && initialization, `segment ${number} did not produce playable video`);
    assert(Math.abs(video.startPTS-number*60) < 0.001, `segment ${number} starts at ${video.startPTS}`);
    assert(Math.abs(video.endPTS-(number+1)*60) < 0.001, `segment ${number} ends at ${video.endPTS}`);
    assert.equal(video.dropped, 0, `segment ${number} dropped frames`);
    assert.deepEqual(errors, [], 'Connect transmuxer reported errors');
    const media = [Buffer.from(video.data1), Buffer.from(video.data2)];
    if (seeking) fs.writeFileSync(path.join(directory, `seek-${number}.mp4`), Buffer.concat([initialization, ...media]));
    else stream.push(...(position ? [] : [initialization]), ...media);
    console.log(`${seeking ? 'Seek' : 'Stream'} ${number}: ${video.startPTS}–${video.endPTS}s, ${video.nb} frames, no errors`);
  }
  if (!seeking) fs.writeFileSync(path.join(directory, 'stream.mp4'), Buffer.concat(stream));
}

check([0, 1, 2, 3], false);
check([3, 1, 0, 2, 3], true);
