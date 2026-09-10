# Patch 0002 — call-local arena for triangulation graph objects

| | |
|---|---|
| **Upstream** | Clipper2 2.0.1, `src/clipper.triangulation.cpp` |
| **Symptom** | thousands of individually allocated vertices, edges, triangles and adjacency buffers per decomposition |
| **Found** | ImCut::Geometry V8.1 WP5 allocation-stage profile |
| **Applied** | yes — `grep -n "ImCut patch 0002"` locates the arena owner |

This is a documented local performance patch to vendored code. It does not change the
triangulation algorithm, ordering, predicates or output conversion.

## Root cause

`Delaunay` creates every `Vertex2`, `Edge` and `Triangle` with a separate `new`. Every
vertex also owns a separate `std::vector<Edge*>`. These objects all have exactly the
lifetime of one `Delaunay::Execute` call and are referenced through stable pointers until
`CleanUp`; individual allocation and deallocation therefore add allocator traffic without
providing useful independent lifetime management.

The V8.1 stage probe measured 11,638 allocations in triangulation out of 17,291 total
decomposition allocations after the fixed-triangle output change. Triangulation was the
dominant avoidable allocation source.

## Change

`Delaunay` now owns one `std::pmr::monotonic_buffer_resource`:

- graph objects are allocated with `std::pmr::polymorphic_allocator` and constructed with
  `std::construct_at`;
- `Vertex2::edges` is a `std::pmr::vector<Edge*>` backed by the same resource;
- pointer stability and all existing pointer relationships are preserved;
- `CleanUp` invokes every non-trivial object destructor, clears the pointer indexes, then
  releases the arena in one operation;
- the arena is a non-static member, so it is call-local through the stack-owned
  `Delaunay`: no global state, thread-local retention or cross-query reuse was added.

The backend adapter already translates `std::bad_alloc` to
`TriangulationStatus::OutOfMemory`; this behavior is unchanged.

## Rejected alternative

Replacing the object vectors with `std::deque` pools retained stable pointers but created
more heap blocks. It regressed the measured decomposition from 17,291 allocations /
1,659,949 bytes to 17,322 allocations / 1,741,936 bytes. That prototype was fully removed;
`TestEvidenceV8_1/78_wp5_pools_alloc.txt` is retained as negative evidence.

## Verification and result

With the arena alone, triangulation allocations fell from 11,638 to 3,457 and complete
decomposition allocations fell from 17,291 to 9,110. The final WP5 composition, including
the exact flat lattice index, records 7,744 allocations and 1,359,393 bytes versus the WP3
baseline of 18,657 allocations and 1,665,025 bytes.

Evidence:

- `79_wp5_arena_build.txt`, `80_wp5_arena_*.txt`, `81_wp5_arena_alloc.txt`;
- `82_wp5_arena_decomp.txt`;
- `89_wp5_final_alloc.txt`, `90_wp5_final_decomp.txt`;
- `96_wp5_full_suite.txt`: 460 cases / 152,749 assertions / 0 failures.

On a Clipper2 upgrade, re-audit ownership and cleanup before reapplying. If upstream has
adopted equivalent lifetime grouping, prefer the upstream implementation.
