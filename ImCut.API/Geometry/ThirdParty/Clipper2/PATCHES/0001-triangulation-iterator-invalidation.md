# Patch 0001 — self-loop edges corrupt `Delaunay::MergeDupOrCollinearVertices`

| | |
|---|---|
| **Upstream** | Clipper2 2.0.1, `src/clipper.triangulation.cpp` |
| **Symptom** | heap-use-after-free, reachable from ordinary kernel input |
| **Found** | ImCut::Geometry Hardening V5, corpus fuzz probe (`audit-v5/probe_fuzz.cpp`, `Z3.MutationProperties`), mutant 1602 |
| **Confirmed by** | AddressSanitizer, `audit-v5/evidence-v5/40_asan_fuzz.txt` |
| **Applied** | yes — `grep -n "ImCut patch 0001"` finds every site |

`ThirdParty/` is not edited as a matter of policy. This is the documented exception the
policy provides for. The justification, including two attempts that did **not** work, is
below.

## What goes wrong

`MergeDupOrCollinearVertices` opens with:

```cpp
// note: this procedure may add new edges and change the
// number of edges connected with a given vertex, but it
// won't add or delete vertices (so it's safe to use iterators)
```

True of `allVertices`, which the outer loop walks. False of the per-vertex `edges`
vectors, which the inner loops walk.

The merge step rewrites the endpoints of `v2`'s edges to point at `v1`:

```cpp
if (e->vB == v2) e->vB = v1;
else e->vT = v1;
```

For an edge that *joined* the two coincident vertices, both endpoints end up at `v1` — a
**self-loop**, `vB == vT == v1`. `std::copy` then moves it into `v1->edges`.

The collinear-split loop that follows filters on `vB == v1`, which a self-loop passes. It
calls `SplitEdge(longE, shortE)`, whose `newT` / `oldT` are the *tops* of those edges —
normally not `v1`, but for a self-loop exactly `v1`. `SplitEdge` then does
`newT->edges.push_back(...)` and `CreateEdge(newT, oldT, ...)`, and `CreateEdge` does
`v1->edges.push_back(res)`.

That `push_back` reallocates the vector the loop is iterating. ASan:
`READ of size 8` on freed memory at `clipper.triangulation.cpp:644`.

## Two fixes that were tried and rejected

**1. Remove the trigger in the adapter — did not work.**
`ToClipperWithoutZeroLengthEdges` in `Polygon/Clipper2Backend.cpp` strips consecutive
duplicate ring points before triangulating, on the reasoning that only consecutive points
get an edge between them. The crash persisted: coincident vertices also arise *inside* the
triangulator, because `SplitEdge` itself calls `CreateEdge(newT, oldT)` and those two can
be distinct-but-coincident. The precondition is not expressible on the input, so no amount
of pre-cleaning at the boundary can guarantee it.

The adapter dedup was kept anyway — a zero-length edge carries no geometry and feeding one
to a triangulator is meaningless work — but it is hygiene, not the fix.

**2. Iterators to indices alone — made it worse.**
Replacing the iterators with indices removes the dangling read, and it converted the
use-after-free into an **unbounded loop**: the new edge that `CreateEdge` appends also has
its bottom at `v1`, so it re-qualifies for splitting, which appends another. Measured on
the same fuzz run: 241 s of CPU and **12.8 GB** resident before it was killed. A crash
replaced by a memory exhaustion is not a fix.

## The change that works

Skip self-loop edges in both collinear-split loops. A zero-length edge has no direction,
so asking whether it is collinear with another edge is meaningless; excluding it removes
work rather than answers. With self-loops excluded, `newT` and `oldT` can never be `v1`,
nothing appends to `v1->edges`, and the original iterator code would in fact have been
safe.

The index form is kept as well, so that a future upstream change which does append cannot
silently resurrect the dangling-iterator failure.

```diff
-      for (auto itE = v1->edges.begin(); itE != v1->edges.end(); ++itE)
+      for (size_t iE = 0; iE < v1->edges.size(); ++iE)
       {
-        if (IsHorizontal(*(*itE)) || (*itE)->vB != v1) continue;
-        for (auto itE2 = itE + 1; itE2 != v1->edges.end(); ++itE2)
+        if (IsHorizontal(*v1->edges[iE]) || v1->edges[iE]->vB != v1 ||
+            v1->edges[iE]->vT == v1) continue;
+        for (size_t iE2 = iE + 1; iE2 < v1->edges.size(); ++iE2)
         {
-          auto e1 = *itE, e2 = *itE2;
-          if (e2->vB != v1 || e1->vT->pt.y == e2->vT->pt.y ||
+          auto e1 = v1->edges[iE], e2 = v1->edges[iE2];
+          if (e2->vT == v1 ||
+              e2->vB != v1 || e1->vT->pt.y == e2->vT->pt.y ||
```

## Verification

| | before | after |
|---|---|---|
| `audit_v5_probes.exe fuzz` | `0xC0000005` at mutant 1602 | `Z2 ok`, `Z3 ok` (4000 mutants), `Z4 ok` |
| peak resident during Z3 | 12.8 GB (index-only attempt) | 22 MB |
| determinism (`Z4`) | — | `stable_over_3_runs=1` |
| suite | 333 cases / 72 562 assertions | unchanged, 0 failures |

`Decomposition` and `HardeningV5_Decomposition` are unchanged and green, so the
triangulation results themselves did not move.

`Z4`'s digest differs from the V4 baseline (`8cba3d…` → `b36111…`). That is expected and
unrelated to this patch: V5 changed the clipping lattice to a local frame and the
containment roles to respect `FillRule`, both of which change output coordinates. What the
gate asserts is stability across runs, and that holds.

## On upgrading Clipper2

Re-apply this patch, or check whether upstream has fixed it.
