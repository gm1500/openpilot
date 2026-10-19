# SLC preferences and offline maps

This feature is integrated into `batmobile`. Sentry route uploads wait while driving, and map requests have priority over upload traffic. Normal driving uploads otherwise retain their existing behavior.

## Settings → SLC

The comma 3X settings panel provides:

| Setting | Options |
| --- | --- |
| SLC this drive | Same enable state as the onroad SLC button |
| New speed limit | Automatic; Lower only; SET to accept |
| After manual override | Resume at the next changed limit; SET only |
| Offset type | km/h or mph, following the device unit setting; Percent |
| Speed offset | −/+ buttons, steps of 5, range −30 to +30; default zero |
| Map area | North America (Canada, U.S., Mexico); individual country |
| Offline maps | Download, Update, Delete, Cancel; progress, storage and download date |
| Automatic map updates | Manual (default); Daily; Weekly |
| Weekly update day | Monday through Sunday; shown when Weekly is selected |

Offsets affect the cruise target, not the displayed posted limit. Targets keep the existing 8–145 km/h bounds and only operate on cars supporting the existing SLC integration. Ordinary cruise +/− buttons retain manual control. New limits retain the existing two-second qualification and fresh-position checks. SET explicitly accepts a fresh map limit; the existing current-speed floor on explicit SET is unchanged.

“Lower only” compares the new adjusted target with the current cruise setting. “SET to accept” waits for explicit map SET (including re-engagement with SET). Manual override preferences govern when automatic following resumes after the driver adjusts cruise. Defaults preserve the prior automatic behavior with no offset.

## Persistent downloads

Maps live under `/data/maps/slc`, outside the openpilot checkout. They survive reboots and branch updates. They are not erased by the normal recording deleter. Both United States and North America include Alaska and Hawaii; individual U.S. states are not offered in the area picker. The country catalog determines other geographic coverage. Regions overlap, and shared groups are downloaded once and retained while another installed region uses them.

Manual bulk downloads and updates run while parked on any connected network reported as unmetered. The existing **Cellular Metered** setting controls cellular eligibility; turn it off to allow cellular downloads. Wi-Fi uses its own metered setting. Eligibility follows `deviceState.networkMetered`, so a toggle change takes effect when hardwared refreshes the network state (normally within about ten seconds). A metered connection, disconnection or driving pauses the job. Completed map groups survive cancellation, connection loss and reboot. An interrupted group restarts, while completed groups are reused by **Download**. **Update** refreshes every group in the selected area. Old groups stay usable until their replacements validate and install atomically. **Delete** removes the selected region and groups no other installed region owns.

Downloads preserve at least 5 GB **and** 10% of total storage for recordings. A large continent can require several GB; there is no reliable fixed size estimate in the provider catalog. Download status reports installed size. Archive paths, sizes, schema, tile bounds and gzip checksums are checked before installation. A manual download error stops the job; Download can resume the retained groups. Scheduled jobs keep progress and retry after one hour.

## Automatic refreshes

Choose **Daily** or **Weekly** and a weekday in Settings → SLC. Manual is the initial setting. Automatic updates refresh every fully downloaded region on **unmetered Wi-Fi while offroad**, even if Cellular Metered is off. They pause when driving starts, Wi-Fi disconnects, or the connection becomes metered. Partially downloaded regions still require an explicit Download.

Daily uses the local calendar day. Weekly uses the most recent selected weekday, catching up at the next parked Wi-Fi connection if the device was off or away. The saved geographic timezone handles daylight-saving changes; UTC is used until a local timezone is known. An invalid system clock cannot schedule downloads. A completed manual update satisfies that period too, and overlapping regions reuse groups already refreshed during the period.

Installed maps and their last successful update date remain available during replacement. Progress and the one-hour failure retry survive restart. Cancel suppresses automatic refreshes for all installed areas for the current calendar period; manual Update remains available. Selecting Manual cancels an automatic job without deleting maps. Cancel a running job before requesting another download or deletion; a rejected request never discards its progress.

## Map behavior

Installed tiles provide an independent local speed-limit fallback when the normal live road cache is unavailable. Disk reads run separately from network requests, so an online timeout cannot block the offline lookup. GPS freshness, road matching, direction, conditional-speed handling and candidate qualification still apply. An untagged or ambiguous road can still have no usable limit.

Live maps work without installing any regional packages. The live query remains 1.5 km on the original Overpass provider, with the same 2 MiB response limit and fresh-position/coverage checks. A fresh online result takes priority. The bulk format does not contain OSM node IDs or complete junction topology, so offline geometry is not used to invent junction connections. Live data continues to supply richer junction/control information. This version therefore supports offline speed limits, not complete offline replacement of every map-derived feature.

When the car leaves the live query circle, the offline matcher uses recent live speed tags for an identical way ID, complete geometry and allowed direction. Tags expire with the existing ten-minute live cache or are replaced by the next live response. Raw legal/advisory, directional and conditional tags are reconciled before matching, so schedules are reevaluated and removed or unsupported limits do not revive old offline values. Offline geometry and coverage remain required; no junction topology is inferred and nothing is written to the installed tiles. Different ways and changed geometry retain their own offline tags.

Source, matched way ID and published limits are logged on changes as `mapd.speed_source`, including transitions to no match. `offline+live` identifies offline geometry using reconciled live speed tags. A newly fetched map is not proof that its speed tags reflect current signs.

Live retrieval allows 30 seconds of server execution and a 45-second socket read timeout, with a 60-second response budget checked after each chunk (a final blocked read is bounded separately by the socket timeout). Normal failed-request waits are 30, then 60 seconds, capped at 60; a server's longer `Retry-After` is respected. A new Wi-Fi/cellular connection can shorten a failed-request wait to five seconds after the last attempt, without bypassing server cooldowns or triggering needless successful-cache refreshes. Requests remain serial and run outside the map publication/control loop.

A successful slow download counts toward the normal 30-second refresh interval, so a car that has already travelled 700 m does not wait another 30 seconds before querying its current position. Municipal queries also use an updated fix after road downloads. Request start, HTTP/error status, duration, header wait, response bytes and road count are logged without coordinates or response bodies. No live cache is written to disk. Server availability and slow responses can still leave gaps; installed regional maps remain optional fallback coverage.

Regional packages use the public `map-data.pfeifer.dev` service used by the upstream map downloader. Availability and source age depend on that provider. “Last update” means the local download date. Map data © OpenStreetMap contributors, under the Open Database License; see `openpilot/third_party/offline_maps/README.md` for provenance.

## Install / update

Run while parked with stable power. The helper updates `batmobile`, updates submodules and LFS assets, rebuilds the native parameter registry without build-cache reuse, verifies the new keys, checks the branch's AGNOS requirement, and reboots only after success. It never changes Sentry enable or shutdown preferences and never deletes footage or installed maps. It refuses tracked local changes or a pending shutdown.

```sh
cd /data/openpilot &&
git fetch origin batmobile &&
git show FETCH_HEAD:tools/slc_update.sh > /tmp/slc-update.sh &&
tmux new-session -s slc-update bash -ic 'bash /tmp/slc-update.sh batmobile; exec bash'
```

If SSH disconnects during the build, reconnect with `tmux attach -t slc-update`. The log is `/data/slc-update.log`. A successful reboot ends the SSH/tmux session. The native rebuild is required for these new settings; rebooting an unrecompiled old registry is insufficient.

Validation includes packed map decoding, interrupted/corrupt installs, restart persistence, shared-region deletion, storage reserves, map fallback, speed-tag reconciliation, and cruise override policies. Real installed-tile/GPS replay verifies the reported same-way disagreement; future provider data and changed road conditions still require road validation.
