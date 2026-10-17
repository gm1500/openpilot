# SLC preferences and offline maps

This feature is on the `slc-offline-maps` test branch. It includes the `batmobile` upload-priority fix: Sentry route uploads wait while driving, and map requests have priority over upload traffic. Normal driving uploads otherwise retain their existing behavior.

## Settings → SLC

The comma 3X settings panel provides:

| Setting | Options |
| --- | --- |
| SLC this drive | Same enable state as the onroad SLC button |
| New speed limit | Automatic; Lower only; SET to accept |
| After manual override | Resume at the next changed limit; SET only |
| Offset type | Speed units (km/h or mph); Percent |
| Speed offset | −/+ buttons, steps of 5, range −30 to +30; default zero |
| Map area | North America (Canada, U.S., Mexico); individual country; U.S. state |
| Offline maps | Download, Update, Delete, Cancel; progress, storage and download date |

Offsets affect the cruise target, not the displayed posted limit. Targets keep the existing 8–145 km/h bounds and only operate on cars supporting the existing SLC integration. Ordinary cruise +/− buttons retain manual control. New limits retain the existing two-second qualification and fresh-position checks. SET explicitly accepts a fresh map limit; the existing current-speed floor on explicit SET is unchanged.

“Lower only” compares the new adjusted target with the current cruise setting. “SET to accept” waits for explicit map SET (including re-engagement with SET). Manual override preferences govern when automatic following resumes after the driver adjusts cruise. Defaults preserve the prior automatic behavior with no offset.

## Persistent downloads

Maps live under `/data/maps/slc`, outside the openpilot checkout. They survive reboots and branch updates. They are not erased by the normal recording deleter. North America includes Alaska and Hawaii; the country catalog determines other geographic coverage. Regions overlap, and shared groups are downloaded once and retained while another installed region uses them.

Bulk downloads run only while parked on unmetered Wi-Fi. Completed map groups survive cancellation, Wi-Fi loss and reboot. An interrupted group restarts, while completed groups are reused by **Download**. **Update** refreshes every group in the selected area. Old groups stay usable until their replacements validate and install atomically. **Delete** removes the selected region and groups no other installed region owns.

Downloads preserve at least 5 GB **and** 10% of total storage for recordings. A large continent can require several GB; there is no reliable fixed size estimate in the provider catalog. Download status reports installed size. Archive paths, sizes, schema, tile bounds and gzip checksums are checked before installation. An error stops the job instead of repeatedly retrying a bad response; Download can resume the retained groups.

## Map behavior

Installed tiles provide an independent local speed-limit fallback when the normal live road cache is unavailable. Disk reads run separately from network requests, so an online timeout cannot block the offline lookup. GPS freshness, road matching, direction, conditional-speed handling and candidate qualification still apply. An untagged or ambiguous road can still have no usable limit.

The normal 1.5 km live map cache, provider and request timing remain unchanged. A fresh online result takes priority. The bulk format does not contain OSM node IDs or complete junction topology, so offline geometry is not used to invent junction connections. Live data continues to supply richer junction/control information. This version therefore supports offline speed limits, not complete offline replacement of every map-derived feature.

Regional packages use the public `map-data.pfeifer.dev` service used by the upstream map downloader. Availability and source age depend on that provider. “Last update” means the local download date. Map data © OpenStreetMap contributors, under the Open Database License; see `openpilot/third_party/offline_maps/README.md` for provenance.

## Install / update

Run while parked with stable power. The helper switches to `slc-offline-maps`, updates submodules and LFS assets, rebuilds the native parameter registry without build-cache reuse, verifies the new keys, checks the branch's AGNOS requirement, and reboots only after success. It never changes Sentry enable or shutdown preferences and never deletes footage or installed maps. It refuses tracked local changes or a pending shutdown.

```sh
cd /data/openpilot &&
git fetch origin slc-offline-maps &&
git show FETCH_HEAD:tools/slc_update.sh > /tmp/slc-update.sh &&
tmux new-session -s slc-update bash -ic 'bash /tmp/slc-update.sh; exec bash'
```

If SSH disconnects during the build, reconnect with `tmux attach -t slc-update`. The log is `/data/slc-update.log`. A successful reboot ends the SSH/tmux session. The native rebuild is required for these new settings; rebooting an unrecompiled old registry is insufficient.

Validation includes packed map decoding, interrupted/corrupt installs, restart persistence, shared-region deletion, storage reserves, map fallback, and cruise override policies. Physical comma 3X download/display behavior and road validation still need device testing before merging this feature into `batmobile`.
