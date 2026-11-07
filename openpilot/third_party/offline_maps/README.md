# Offline road data

The packed Cap'n Proto schema and region catalog are from [pfeiferj/mapd](https://github.com/pfeiferj/mapd/tree/8e5eee69f7904ed98eed91ceff4ab5919bec0e3c), under the included MIT license. Only Go-specific schema annotations have been removed. This integration reads data; it does not run the upstream mapd daemon.

Regional archives are fetched over HTTPS from `https://map-data.pfeifer.dev/offline/<latitude>/<longitude>.tar.gz`. Archive groups span two degrees, containing quarter-degree packed road tiles. Data availability and age are controlled by the upstream publisher. The catalog describes geographic coverage, not guaranteed speed-limit completeness.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright), available under the Open Database License. Preserve this attribution in the download settings. Bulk tiles contain road geometry, way IDs, speeds, directional and conditional fields when provided. They do not contain OSM node IDs, so geometry coincidence must never be treated as proven intersection connectivity.

