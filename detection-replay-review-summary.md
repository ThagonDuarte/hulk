# Detection Replay Review Summary

## Scope

Reviewed commit `86ac10cbc` (`add field and robot poses to detection replay`),
covering changes since `b36d794d7`.

The review examined the detection output contract, cache schema and migration,
runner synchronization, field-feature and robot-pose rendering, documentation,
runtime behavior, and API design.

## Assessment

No blocking issues were found. Field-feature and robot-pose outputs are captured,
timestamp-aligned, filtered, cached, loaded, and rendered coherently. The optional
output handling preserves object-only and legacy pose-model compatibility. No
concurrency or lifecycle defect was identified.

## Non-Blocking Follow-Ups

1. Strengthen relocated recording identity.

   `RecordingFingerprint::matches_relocated` compares only file size, modification
   time, and format version. A path-independent content digest would prevent two
   recordings with matching metadata from sharing predictions accidentally.

2. Preserve legacy payload identity during migration.

   Legacy model caches are migrated to the current payload identity even though
   they have no robot-pose or field-feature sidecars. Keeping the legacy identity,
   or starting a separate current-version run, would make supplemental rerendering
   more predictable.

3. Record expected optional outputs in the manifest.

   Output availability is currently inferred from sidecar-file presence. Persisting
   model capabilities would distinguish an object-only cache from an incomplete
   transfer that omitted every supplemental sidecar.

4. Reject supplemental chunks at `index >= expected_chunks`.

   The current checks use `index > expected_chunks`, allowing one stale chunk past
   the valid range.

5. Introduce a frame-oriented loaded prediction type.

   `LoadedPredictionChunk` exposes predictions, field features, and robot poses as
   parallel vectors. A single per-frame aggregate would encode alignment in the
   type system and reduce cloning and per-frame `Arc` allocation.

6. Avoid eagerly decoding all supplemental history on cache open.

   Resume currently reads and validates every field-feature and robot-pose chunk,
   although only the final partial chunk is retained. Lazy validation or metadata
   checks would improve startup time for long recordings.

7. Separate cache relocation from supplemental-output support in history.

   Cross-machine cache relocation is independently useful and riskier than the
   pose/field rendering work. A separate commit would make it easier to review and
   revert independently.

8. Consolidate duplicated supplemental-cache and skeleton-rendering logic.

   Field and robot sidecar handling follows nearly identical persistence and load
   paths, while person and robot skeleton drawing repeat the same projection and
   clipping workflow. Shared helpers would reduce future drift.

## Open Questions

- Was the field-feature-only payload version 2 distributed outside repository
  history? If not, its compatibility scaffolding may be unnecessary.
- Is metadata-only relocated-cache matching an accepted tradeoff for this viewer?

## Validation Note

This was a static, read-only code review. No tests were run as part of the review.
