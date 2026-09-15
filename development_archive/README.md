# Development snapshot

`dev` preserves this model project's complete development assets, including all
design revisions, scripts, reports, generated previews, upstream working files
and the three externally referenced original CAD files in `original_cad/`.
Local Python environments and interpreter caches are excluded.

`vendor_metadata/*.git.tar.gz` preserves the locally available metadata of the
three nested vendor repositories; their working files are ordinary tracked
files in this snapshot, not empty submodule references. Partial upstream clones
remain partial; the archives do not claim to contain all remote history.

`snapshot_manifest.json` lists the archived files with SHA-256 digests and sizes.
The manifest itself is excluded from its own listing. The adjacent glove
application, delivery bundle, and skill handoff are separate projects and are
not modified or incorporated here. No remote publication is performed.

`main` is the cleaned runtime/model delivery derived from this snapshot. Use
`git switch dev` to return to the development assets, or `git restore --source
dev -- <path>` to recover an individual file.
