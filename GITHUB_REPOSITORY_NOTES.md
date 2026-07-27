# Private GitHub repository notes

This folder is prepared to become the root of a new, independent private
repository. It has not been initialized, committed, or pushed.

## Git LFS

Git Large File Storage keeps a small pointer in normal Git history while the
actual binary is stored in GitHub's LFS object storage. A normal clone with
Git LFS installed resolves the pointer and downloads the real file.

`.gitattributes` is configured to use LFS for videos, learned-model formats,
NPZ motion data, and STL meshes. This is particularly useful for the
metadata-sanitized MOV, which is approximately 89 MiB, and for future policy
checkpoints.

Each collaborator should install Git LFS and run:

```text
git lfs install
```

Git LFS is not encryption. Repository visibility and collaborator permissions
still control who can retrieve the files.

## Commit safeguards

`.gitignore` excludes common credential/key files, local caches, generated
ZIP files, and private sanitization work. It reduces accidental commits but
does not replace a secret scan or code review.

Recommended repository settings:

- Private visibility
- Only named project collaborators
- No public forks
- Review changes before merging
- Re-run the share-security scan before adding new videos, logs, or robot
  configuration captures
