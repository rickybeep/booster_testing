# Pre-share security and privacy review

Review date: 2026-07-27 EDT

Scope:

`K1_TEAM_PROJECT_BRIEF_20260727/`

## Confirmed clean

- No API keys, access tokens, passwords, private keys, cloud credentials, or
  authenticated URLs were found.
- No `.env`, credential, SSH-key, cookie, wallet, or secret-named files were
  found.
- No unexpected hidden files, symbolic links, junctions, Git metadata, or
  repository history are present. The only intentional dotfiles are
  `.gitignore` and `.gitattributes`.
- No Beeple team email addresses, telephone numbers, Windows user profiles,
  workstation names, home-directory paths, or current-repository paths were
  found in text or binary string scans.
- The apparent dotted-number matches in the robot documents are firmware
  version `1.7.0.7`, not IP addresses.
- The GVHMR `.pt` contains tensors/dictionaries and no sensitive embedded
  strings.
- The NPZ files contain numeric arrays plus joint/body names; they contain no
  local paths or personal strings.
- The included JPEG and PNG contain no EXIF, GPS, camera, or creator metadata.
- Generated MP4 files contain ordinary codec/encoder tags only.
- STL headers contain no paths, usernames, or author/device metadata.

## Raw-video remediation

The original iPhone MOV contained:

- Precise GPS coordinates and horizontal accuracy
- Capture timestamp
- Phone make/model and software version
- Two audio streams
- Multiple QuickTime metadata streams

The copy in this handoff was remuxed before sharing. The handoff MOV now
contains one H.264 video stream, no audio, no metadata streams, and no
GPS/device/capture tags. The encoded video-stream SHA-256 remained identical
before and after sanitization:

```text
2fb76dd8b7b27506fe721914173a28d6f92f9c5b6bf8b926c73e85ac150fa589
```

The unsanitized source remains outside this handoff folder and must not be
committed to the collaboration repository.

## Intentional information that remains

- The raw/normalized video and derived previews visibly identify the human
  performer and show the recording interior. No screens, documents, addresses,
  or obvious confidential objects were seen in the reviewed frames. Obtain
  the performer's consent before sharing; omit these files or create a
  face-blurred copy if that consent is not available.
- Three stock Booster URDF files retain the public email address in the
  standard SolidWorks-to-URDF exporter attribution comment. This is
  third-party attribution, not a Beeple contact or credential.
- Robot edition, firmware version, interface findings, failure messages, and
  upstream commit identifiers remain intentionally because they are needed
  for the technical collaboration.

## Sharing recommendation

Use a separate **private** repository with access limited to the project
participants. Do not make it public without another content and license
review. Git LFS changes how large binaries are stored; it is not encryption
and does not replace repository access controls.
