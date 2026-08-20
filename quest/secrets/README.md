# WebRTC signalling secrets

Two files, both **gitignored**, both required by `--backend webrtc`. They are
mounted read-only at `/secrets`; nothing is baked into the image.

| File | What it is |
| --- | --- |
| `serviceAccountKey.json` | Google service account with Firestore access — the offer/answer rendezvous |
| `signalingSettings.json` | `robotID`, `password`, `turn_server_url`, `turn_server_username`, `turn_server_password` |

Copy them from the giava checkout:

```bash
cp ~/giava/interbotix_ws/src/av_aloha/data_collection_scripts/serviceAccountKey.json  quest/secrets/
cp ~/giava/interbotix_ws/src/av_aloha/data_collection_scripts/signalingSettings.json quest/secrets/
```

`robotID` and `password` must match what the Unity app on the headset is set to,
or both ends will sit waiting on different Firestore documents and neither will
report an error — the offer is simply never answered.
