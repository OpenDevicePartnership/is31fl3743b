# AGENTS.md

Canonical guidance for AI coding agents (and human contributors) working in
this repository. Keep this file in sync with reality: if you change build
commands, lints, MSRV, features, or layout, update this file in the same
commit.

## What this crate is

- **Crate name:** `is31fl3743b-driver` (see `Cargo.toml`).
- **Purpose:** Platform-agnostic Rust driver for the Lumissil
  [IS31FL3743B](https://lumissil.com/assets/pdf/core/IS31FL3743B_DS.pdf)
  18×11 LED matrix controller, built on top of the `embedded-hal` and
  `embedded-hal-async` SPI traits.
- **Posture:** `#![no_std]` library (see `src/lib.rs` line 13:
  `#![cfg_attr(not(test), no_std)]`). `std` is only enabled implicitly
  while running unit tests on the host.
- **Async model:** the crate uses the [`maybe-async`](https://crates.io/crates/maybe-async)
  crate to expose a single source tree as either an async API (default)
  or a blocking API (`is_blocking` feature). Do **not** write parallel
  async/blocking implementations by hand — annotate with `maybe-async`
  attributes instead.
- **License:** MIT (`LICENSE`). `unsafe_code` is `forbid`-ed crate-wide
  (`Cargo.toml` `[lints.rust]`).

## Repository layout

```
.
├── Cargo.toml              # crate manifest, MSRV, features, lint config
├── Cargo.lock              # committed; CI runs with --locked
├── README.md               # included into the crate root docs via include_str!
├── LICENSE                 # MIT
├── rust-toolchain.toml     # stable channel + embedded targets pinned
├── rustfmt.toml            # formatting rules (nightly rustfmt features)
├── deny.toml               # cargo-deny configuration
├── CONTRIBUTING.md         # ODP-wide contribution rules
├── CODE_OF_CONDUCT.md
├── SECURITY.md
├── CODEOWNERS
├── .github/
│   ├── copilot-instructions.md   # AI-specific commit/attribution rules
│   └── workflows/
│       ├── check.yml             # fmt, clippy, semver, doc, hack, deny, msrv
│       └── nostd.yml             # cross-check for thumbv8m.main-none-eabihf
├── src/
│   ├── lib.rs              # public driver API (Is31fl3743b, SWx, CSy, …)
│   └── registers.rs        # register-level types built with `bilge` bitfields
└── examples/               # separate crate (not part of the library workspace)
    ├── Cargo.toml          # is31fl3743b-driver-examples (path-deps on `..`)
    ├── build.rs
    ├── .cargo/config.toml  # Embassy/STM32F411 build config
    └── src/bin/
        ├── breathing.rs
        └── one_by_one_shared.rs
```

The `examples/` directory is a **separate crate** with its own `Cargo.toml`
and is built for `thumbv7em-none-eabihf` against `embassy-stm32`. It is
intentionally *not* part of a Cargo workspace with the library — running
`cargo` from the repo root touches only the library crate.

## Building and testing

All commands below have been run from the repo root on a stable toolchain
unless noted. CI commands are taken verbatim from `.github/workflows/`.

### Library (host)

| Purpose                       | Command                                                                 | Source                  |
|-------------------------------|-------------------------------------------------------------------------|-------------------------|
| Compile check                 | `cargo check --locked`                                                  | `check.yml` `msrv` job  |
| Unit + doctests               | `cargo test --locked`                                                   | local convention        |
| Generate API docs             | `RUSTDOCFLAGS=--cfg docsrs cargo doc --no-deps --all-features --locked` | `check.yml` `doc` job   |
| Feature-powerset build check  | `cargo hack --feature-powerset check --locked`                          | `check.yml` `hack` job  |
| Dependency policy             | `cargo deny --all-features --locked check`                              | `check.yml` `deny` job  |
| SemVer surface check          | `cargo semver-checks` (via `obi1kenobi/cargo-semver-checks-action@v2`)  | `check.yml` `semver` job|

On Windows PowerShell, set `RUSTDOCFLAGS` with
`$env:RUSTDOCFLAGS = '--cfg docsrs'` before invoking `cargo doc`.

### no_std cross-check

CI builds the library for a Cortex-M33 target without default features:

```
rustup target add thumbv8m.main-none-eabihf
cargo check --target thumbv8m.main-none-eabihf --no-default-features --locked
```

The `rust-toolchain.toml` already lists the embedded targets the project
cares about (`thumbv6m`/`thumbv7m`/`thumbv7em`/`thumbv7em-hf`/
`thumbv8m.main-hf`, `riscv32imac-unknown-none-elf`, `wasm32-unknown-unknown`),
so `rustup target add` is usually redundant locally — `rust-toolchain.toml`
will install them on first use.

### Formatting and lints

```
cargo +nightly fmt --check     # CI runs nightly rustfmt
cargo +nightly fmt             # apply formatting
cargo clippy --locked --all-targets
```

CI runs `clippy` on **both `stable` and `beta`** (`check.yml` `clippy`
job) via `giraffate/clippy-action`. New lints introduced in beta are
expected to surface here; treat new warnings as failures.

`rustfmt.toml` requires nightly rustfmt because it uses the unstable
options `group_imports = "StdExternalCrate"` and
`imports_granularity = "Module"`. `max_width = 120`.

### Examples crate

```
cd examples
cargo build --release --bin breathing
cargo build --release --bin one_by_one_shared
```

The examples target `thumbv7em-none-eabihf` (configured in
`examples/.cargo/config.toml`) and pull patched `embassy-*` crates from
a pinned git revision (`examples/Cargo.toml` `[patch.crates-io]`). They
are **not** built by CI; treat them as user documentation, not as a
regression suite.

## Code conventions

- **MSRV:** `1.75` (`Cargo.toml` `rust-version = "1.75"`, matched by the
  `msrv` matrix in `check.yml`). Do not use features introduced after
  Rust 1.75 in library code, and do not bump `rust-version` casually —
  it is a public contract.
- **Edition:** 2021.
- **`unsafe_code = "forbid"`** and **`missing_docs = "deny"`** are
  crate-wide (`Cargo.toml`). Every public item needs a doc comment;
  every public re-export from `registers` is documented too.
- **Clippy floors** (`Cargo.toml` `[lints.clippy]`):
  - `correctness`, `suspicious`, `perf`, `style` are **`forbid`**.
  - `pedantic` is **`deny`**. Pedantic lints fire on this code base, so
    do not introduce new pedantic violations even if they look stylistic
    (e.g. `doc_markdown`, `must_use_candidate`, `needless_pass_by_value`).
- **Async/blocking duality:** code lives in one source tree and is
  switched via `cfg(feature = "is_blocking")` plus `maybe-async`
  attributes (see `src/lib.rs` lines 17–26 for the import switch and
  the `maybe-async` usage throughout the impl blocks). When adding a
  new method that performs SPI I/O, use the existing pattern — do not
  duplicate methods.
- **Register layer:** `src/registers.rs` uses the [`bilge`](https://crates.io/crates/bilge)
  bitfield macros. Add new registers there, not inline in `lib.rs`.
- **Imports:** grouped `Std / External / Crate` and collapsed to module
  granularity by `rustfmt` config — let rustfmt handle ordering rather
  than hand-sorting.
- **Errors:** SPI operations bubble up `SpiDevice::Error` values via
  generic `E` parameters; the crate does not define its own error enum
  for I/O. Domain validation uses `()`-style error returns (see
  `TryFrom<u8> for SWx` in `src/lib.rs`).

## Driver / hardware specifics

- The IS31FL3743B is an 18×11 (CSy × SWx) LED matrix controller spoken
  to over SPI. Switch columns are `SW1..=SW11`, current source rows are
  `CS1..=CS18`. The `SWx` and `CSy` enums in `src/lib.rs` are the only
  supported coordinate inputs; do not accept raw `u8` in public APIs
  that take coordinates.
- Register ranges are encoded as constants at the top of `src/lib.rs`
  (`SW_MIN/MAX`, `CS_MIN/MAX`, `OPEN_REG_*`, `LED_REG_*`). Reuse those
  rather than hard-coding magic numbers.
- The driver supports two construction modes:
  - `Is31fl3743b::new(spi)` — single device.
  - `Is31fl3743b::new_with_sync([spi0, spi1, …])` — multi-device with
    the first entry acting as the SYNC master. The returned wrapper
    implements `Deref`/`DerefMut`/`Index`/`IndexMut` for per-device
    access.
- The `preserve_registers` feature changes the semantics of
  `detect_shorts` / open-detect helpers (they save and restore PWM /
  scaling state). It is **off by default** to keep the no-feature
  binary size small. If you add a code path that reads/writes those
  registers during a diagnostic, gate the save/restore on
  `cfg(feature = "preserve_registers")`.
- `defmt` is **not** a library dependency. The examples pull `defmt` in,
  but the driver itself stays log-free so it can be used in environments
  without a logger.

## Commit & PR conventions

Derived from `.github/copilot-instructions.md`, `CONTRIBUTING.md`, and
the recent history (`git log --pretty=%s`):

- Subject line: capitalized, ≤50 characters, imperative mood
  (e.g. `Add multi-device sync support`, not `Added …`).
- Blank line, then a wrap-at-72 body explaining *what* and *why*, not
  *how*.
- Merges into `main` use the GitHub default merge-commit subject
  (`Merge pull request #N from user/branch`). Do not squash —
  `CONTRIBUTING.md` ("Clean Commit History") explicitly disables
  squashing and asks contributors to curate their own history so that
  **each commit builds without warnings** and typo/format fix-ups are
  squashed into their parents.
- PRs should start as **drafts** and only be marked ready once every
  CI workflow in `.github/workflows/` is green on the PR branch.
- **AI attribution is mandatory.** Every commit that contains AI-assisted
  work must include an `Assisted-by:` trailer of the form
  `Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL1] [TOOL2]` (see
  `.github/copilot-instructions.md`). AI agents **must not** add
  `Signed-off-by:` trailers — only humans can certify the DCO.

Per-commit identity should be set with
`git -c user.name=... -c user.email=... commit ...` rather than via
global `git config`, so contributor identity stays correct in shared
environments.

## What not to do

- Do **not** introduce `unsafe` blocks — `unsafe_code = "forbid"` will
  reject them.
- Do **not** add `Signed-off-by:` trailers from an AI agent.
- Do **not** bypass `maybe-async` by writing hand-rolled async/blocking
  twins of the same method.
- Do **not** depend on `std`, `alloc`, `defmt`, or `log` in the library
  crate. Anything that needs a host runtime belongs under `examples/`
  or under `#[cfg(test)]`.
- Do **not** add a workspace `Cargo.toml` joining `.` and `examples/`.
  The examples crate is deliberately separate because it patches
  `embassy-*` to a pinned git revision; pulling it into a workspace
  would force the patch on everyone.
- Do **not** force-push to `main` or rewrite already-merged history.
- Do **not** raise MSRV (`rust-version`) without a separate, motivated
  commit and a matching update to the `msrv` matrix in `check.yml`.
- Do **not** loosen the `[lints.clippy]` table to silence new warnings —
  fix the code or, if a lint is genuinely wrong here, add a narrowly
  scoped `#[allow(...)]` with a comment.

## How to find more context

- Datasheet: <https://lumissil.com/assets/pdf/core/IS31FL3743B_DS.pdf>
  (linked from `src/lib.rs`). Use it as the source of truth for register
  layouts and timing.
- `embedded-hal` 1.0 SPI traits: <https://docs.rs/embedded-hal/1.0.0>.
- `embedded-hal-async` 1.0 SPI traits: <https://docs.rs/embedded-hal-async/1.0.0>.
- `maybe-async` macro behaviour: <https://docs.rs/maybe-async>.
- `bilge` bitfield macros: <https://docs.rs/bilge>.
- ODP-wide policies live in `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`,
  and `SECURITY.md` at the repo root.

## Incorporated from `.github/copilot-instructions.md`

The following is the full current content of
`.github/copilot-instructions.md`, folded in so this file is a strict
superset. The original file now points at this one as the canonical
source.

> # Copilot Instructions
>
> ## Commit Messages
> - Subject line: capitalized, 50 characters or less, imperative mood (e.g., "Fix bug" not "Fixed bug")
> - Separate subject from body with a blank line
> - Wrap body text at 72 characters
> - Use the body to explain *what* and *why*, not *how*
>
> ## AI Attribution
> Every commit that includes AI-generated or AI-assisted work **must** contain an `Assisted-by` trailer in the commit message:
> ```
> Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL1] [TOOL2]
> ```
> Where:
> - `AGENT_NAME` is the name of the AI tool or framework (e.g., `GitHub Copilot`)
> - `MODEL_VERSION` is the specific model version used (e.g., `claude-opus-4.6`)
> - `[TOOL1] [TOOL2]` are optional specialized analysis tools used (e.g., `coccinelle`, `sparse`, `smatch`, `clang-tidy`)
> Basic development tools (git, cargo, editors) should not be listed.
> AI agents **must** verify their own identity (agent name and model version) before composing the `Assisted-by` trailer — do not assume or hard-code a model name from a previous session.
> AI agents **MUST NOT** add `Signed-off-by` tags. Only humans can certify the Developer Certificate of Origin.

## Model selection & cost discipline

Premium models (Opus, GPT-5 family, "high"/"xhigh" reasoning variants)
cost an order of magnitude more than standard models (Sonnet, Haiku,
mini). Most steps in a typical task do not need premium reasoning,
and over-using premium models wastes credits without improving
outcomes. The rules below apply to *all* model selection: your own
session, sub-agents launched via the `task` tool, and parallel work
launched via `/fleet`.

### Default posture

- **Default to the cheapest model that can do the job.** Reach for a
  premium model only when one of the escalation triggers below is hit.
- **Plan with premium, execute with cheap.** Spend at most one or two
  premium turns on design / planning, then downshift to a cheaper
  model for mechanical execution of the plan.
- **Never bump the model "just in case."** If you cannot articulate
  *why* a cheaper model would fail, use the cheaper model.

### Escalation triggers (use a premium model)

Reach for a premium model when *any* of these are true:

- Cross-module refactor, architectural design, or API design from
  scratch.
- Subtle correctness reasoning: concurrency, lifetimes, `unsafe`,
  FFI ABI, cryptography, safety-critical control paths.
- Debugging a failure that survived one prior cheap-model attempt.
- Reviewing code on a safety-, security-, or money-critical path.
- The diff cannot be predicted in advance — i.e. there is genuine
  creative or design work to do, not just typing.

### De-escalation triggers (use a cheap model)

Use the cheapest available model when *any* of these are true:

- Searching, reading, summarising files or docs.
- Single-file mechanical edits: rename, format, lint fix, dependency
  bump, boilerplate, scaffolding from a known template.
- Generating tests for code that already works.
- Running builds, tests, linters, or other commands where the model
  only needs to report success/failure.
- Routine commits, PR descriptions, changelog entries.
- The diff is essentially predictable before generation.

### Sub-agent routing (the `task` tool)

When delegating with the `task` tool, set `model:` explicitly. Do not
let sub-agents inherit a premium default for cheap work.

| Sub-agent type    | Default model             | Override to                                     |
|-------------------|---------------------------|-------------------------------------------------|
| `explore`         | cheap                     | keep cheap (`claude-haiku-4.5` or `gpt-5-mini`) |
| `task` (run cmd)  | cheap                     | keep cheap                                      |
| `research`        | cheap for breadth         | premium only for the final synthesis            |
| `general-purpose` | match task                | cheap for mechanical work; premium for design   |
| `rubber-duck`     | premium                   | keep premium — this is where reasoning pays off |
| `code-review`     | premium on critical paths | cheap on cosmetic / mechanical diffs            |

### `/fleet` (parallel sub-agents) rules

- Fleet mode multiplies cost by the fleet width. Apply the rules
  above *per worker*, not in aggregate.
- Split a fleet job along complexity lines: route the cheap,
  parallelisable workers (file edits, test runs, doc updates) to a
  cheap model; reserve premium models for the small number of
  workers that need real reasoning.
- If every worker in a fleet would need a premium model, the work is
  probably not a good fit for fleet mode — reconsider the
  decomposition before paying N× premium.

### Session hygiene

- Keep sessions short and focused. Long premium sessions are the
  single largest source of waste because every turn re-processes the
  full history.
- Use `/compact` when the conversation grows long, and `/new` for
  unrelated work.
- Prefer `/ask` for one-off side questions so they don't extend the
  main session.

### When in doubt

Ask: *"If a cheaper model produced the wrong answer here, would I
catch it in seconds (compiler, tests, my own review) or in
weeks (production incident)?"* If the former, use the cheap model
and let the feedback loop do its job.
